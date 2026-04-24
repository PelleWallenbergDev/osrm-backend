#!/usr/bin/env python3
"""
Smoke-test temporal OSRM route behavior over many coordinate pairs.

The script compares five requests for each sampled pair:

  1. static route
  2. depart_at route, which temporalizes durations on the selected route
  3. depart_at + temporal_mode=asymmetric, which uses the experimental
     plain asymmetric temporal search path
  4. depart_at + temporal_mode=asymmetric_optimized, which uses the
     upper-bound-pruned forward asymmetric search path
  5. depart_at + temporal_mode=overlay_asymmetric, which uses the
     clique-backed temporal overlay search path

It classifies pairs as temporal-effect or no-temporal-effect by comparing
duration and geometry against the static baseline, and it flags the important
regressions where static routing succeeds but temporal routing returns NoRoute.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import http.client
import json
import math
import random
import sys
import threading
import time
import urllib.parse


DEFAULT_THESSALONIKI_BBOX = (22.90, 40.58, 23.02, 40.70)
THREAD_LOCAL = threading.local()


class ThreadLocalHTTPSession:
    def __init__(self, timeout):
        self.timeout = timeout
        self.connections = {}

    def close(self, key):
        connection = self.connections.pop(key, None)
        if connection is not None:
            connection.close()

    def get_connection(self, parsed):
        key = (parsed.scheme, parsed.netloc)
        connection = self.connections.get(key)
        if connection is not None:
            return key, connection

        if parsed.scheme == "https":
            connection = http.client.HTTPSConnection(parsed.netloc, timeout=self.timeout)
        elif parsed.scheme == "http":
            connection = http.client.HTTPConnection(parsed.netloc, timeout=self.timeout)
        else:
            raise ValueError(f"unsupported URL scheme: {parsed.scheme}")

        self.connections[key] = connection
        return key, connection

    def get(self, url, headers):
        parsed = urllib.parse.urlsplit(url)
        path = parsed.path or "/"
        if parsed.query:
            path += "?" + parsed.query

        key, connection = self.get_connection(parsed)
        try:
            connection.request("GET", path, headers=headers)
            response = connection.getresponse()
            body = response.read().decode("utf-8", errors="replace")
            return response.status, body
        except (OSError, http.client.HTTPException):
            self.close(key)
            key, connection = self.get_connection(parsed)
            connection.request("GET", path, headers=headers)
            response = connection.getresponse()
            body = response.read().decode("utf-8", errors="replace")
            return response.status, body


def parse_bbox(value):
    parts = [float(part.strip()) for part in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "bbox must be min_lon,min_lat,max_lon,max_lat"
        )
    min_lon, min_lat, max_lon, max_lat = parts
    if min_lon >= max_lon or min_lat >= max_lat:
        raise argparse.ArgumentTypeError("bbox min values must be smaller than max values")
    return min_lon, min_lat, max_lon, max_lat


def parse_departure_range(value):
    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("departure range must be min_timestamp,max_timestamp")
    start, end = parts
    if start > end:
        raise argparse.ArgumentTypeError("departure range start must be <= end")
    return start, end


def haversine_km(a, b):
    lon1, lat1 = a
    lon2, lat2 = b
    radius_km = 6371.0088
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    h = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    return 2.0 * radius_km * math.asin(math.sqrt(h))


def get_session(timeout):
    session = getattr(THREAD_LOCAL, "session", None)
    if session is None:
        session = ThreadLocalHTTPSession(timeout)
        THREAD_LOCAL.session = session
    return session


def open_json(url, timeout, pool_size):
    del pool_size
    session = get_session(timeout)
    started = time.perf_counter()
    try:
        status, text = session.get(url, headers={"User-Agent": "osrm-temporal-smoke"})
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        try:
            payload = json.loads(text)
        except ValueError:
            payload = {"code": "HTTPError", "message": text}
        return status, payload, elapsed_ms
    except (OSError, http.client.HTTPException, ValueError) as error:
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return 0, {"code": "TransportError", "message": str(error)}, elapsed_ms


def route_url(host, start, target, departure, mode):
    coordinates = f"{start[0]:.7f},{start[1]:.7f};{target[0]:.7f},{target[1]:.7f}"
    params = {
        "overview": "full",
        "annotations": "duration",
        "alternatives": "false",
    }
    if departure is not None:
        params["depart_at"] = str(departure)
    if mode in {"asymmetric", "asymmetric_optimized", "overlay_asymmetric"}:
        params["temporal_mode"] = mode

    return f"{host.rstrip('/')}/route/v1/driving/{coordinates}?{urllib.parse.urlencode(params)}"


def nearest_url(host, coord):
    params = urllib.parse.urlencode({"number": "1"})
    return f"{host.rstrip('/')}/nearest/v1/driving/{coord[0]:.7f},{coord[1]:.7f}?{params}"


def route(host, start, target, timeout, pool_size, departure=None, mode="static"):
    status, payload, elapsed_ms = open_json(
        route_url(host, start, target, departure, mode), timeout, pool_size
    )
    code = payload.get("code", "MissingCode")
    result = {
        "http_status": status,
        "code": code,
        "message": payload.get("message", ""),
        "elapsed_ms": elapsed_ms,
        "duration": None,
        "distance": None,
        "geometry": None,
    }

    if code == "Ok" and payload.get("routes"):
        best = payload["routes"][0]
        result["duration"] = float(best.get("duration", 0.0))
        result["distance"] = float(best.get("distance", 0.0))
        result["geometry"] = best.get("geometry")

    return result


def snap(host, coord, timeout, pool_size):
    status, payload, _elapsed_ms = open_json(nearest_url(host, coord), timeout, pool_size)
    if status != 200 or payload.get("code") != "Ok" or not payload.get("waypoints"):
        return None
    location = payload["waypoints"][0].get("location")
    if not location or len(location) != 2:
        return None
    return float(location[0]), float(location[1])


def parse_point_row(row):
    lower = {key.lower(): value for key, value in row.items() if key is not None}
    lon_keys = ("lon", "longitude", "@lon", "x")
    lat_keys = ("lat", "latitude", "@lat", "y")

    lon = next((lower[key] for key in lon_keys if key in lower), None)
    lat = next((lower[key] for key in lat_keys if key in lower), None)
    if lon is not None and lat is not None:
        return float(lon), float(lat)

    values = list(row.values())
    numeric = []
    for value in values:
        try:
            numeric.append(float(value))
        except (TypeError, ValueError):
            pass
    if len(numeric) >= 2:
        return numeric[0], numeric[1]

    return None


def read_points(path):
    with open(path, newline="", encoding="utf-8-sig") as handle:
        sample = handle.read(4096)
        handle.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",\t; ")
            has_header = csv.Sniffer().has_header(sample)
        except csv.Error:
            dialect = csv.excel
            has_header = True

        points = []
        if has_header:
            reader = csv.DictReader(handle, dialect=dialect)
            for row in reader:
                point = parse_point_row(row)
                if point is not None:
                    points.append(point)
        else:
            reader = csv.reader(handle, dialect=dialect)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    points.append((float(row[0]), float(row[1])))
                except ValueError:
                    continue

    if len(points) < 2:
        raise RuntimeError(f"{path} must contain at least two points")
    return points


def random_point(bbox):
    min_lon, min_lat, max_lon, max_lat = bbox
    return random.uniform(min_lon, max_lon), random.uniform(min_lat, max_lat)


def sample_pair(points, bbox, max_straight_line_km, max_attempts=500):
    for _ in range(max_attempts):
        if points:
            start, target = random.sample(points, 2)
        else:
            start, target = random_point(bbox), random_point(bbox)

        if max_straight_line_km <= 0 or haversine_km(start, target) <= max_straight_line_km:
            return start, target

    raise RuntimeError("could not sample a pair within max straight-line distance")


def differs(reference, candidate, duration_tolerance):
    if reference["code"] != "Ok" or candidate["code"] != "Ok":
        return False
    if reference["geometry"] != candidate["geometry"]:
        return True
    return abs(reference["duration"] - candidate["duration"]) > duration_tolerance


def write_csv_header(writer):
    writer.writerow(
        [
            "index",
            "start_lat",
            "start_lon",
            "target_lat",
            "target_lon",
            "departure",
            "departure_minutes",
            "static_code",
            "depart_code",
            "asymmetric_code",
            "asymmetric_optimized_code",
            "overlay_code",
            "static_duration_seconds",
            "static_duration_minutes",
            "depart_duration_seconds",
            "depart_duration_minutes",
            "asymmetric_duration_seconds",
            "asymmetric_duration_minutes",
            "asymmetric_optimized_duration_seconds",
            "asymmetric_optimized_duration_minutes",
            "overlay_duration_seconds",
            "overlay_duration_minutes",
            "depart_effect",
            "asymmetric_effect",
            "asymmetric_optimized_effect",
            "overlay_effect",
            "static_ok_asymmetric_failed",
            "static_ok_asymmetric_optimized_failed",
            "static_ok_overlay_failed",
            "asymmetric_ok_overlay_failed",
            "asymmetric_optimized_ok_overlay_failed",
            "asymmetric_message",
            "asymmetric_optimized_message",
            "overlay_message",
            "static_ms",
            "depart_ms",
            "asymmetric_ms",
            "asymmetric_optimized_ms",
            "overlay_ms",
        ]
    )


def seconds_to_minutes(value):
    if value is None:
        return None
    return value / 60.0


def percentile(values, ratio):
    if not values:
        return None
    if len(values) == 1:
        return values[0]

    position = (len(values) - 1) * ratio
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]

    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def compute_latency_stats(values):
    if not values:
        return None

    ordered = sorted(values)
    return {
        "count": len(ordered),
        "mean": sum(ordered) / len(ordered),
        "median": percentile(ordered, 0.5),
        "p95": percentile(ordered, 0.95),
    }


def write_csv_row(
    writer,
    index,
    start,
    target,
    departure,
    static,
    depart,
    asymmetric,
    asymmetric_optimized,
    overlay,
    depart_effect,
    asym_effect,
    asym_optimized_effect,
    overlay_effect,
):
    static_ok_asym_failed = static["code"] == "Ok" and asymmetric["code"] != "Ok"
    static_ok_asym_optimized_failed = (
        static["code"] == "Ok" and asymmetric_optimized["code"] != "Ok"
    )
    static_ok_overlay_failed = static["code"] == "Ok" and overlay["code"] != "Ok"
    asymmetric_ok_overlay_failed = asymmetric["code"] == "Ok" and overlay["code"] != "Ok"
    asymmetric_optimized_ok_overlay_failed = (
        asymmetric_optimized["code"] == "Ok" and overlay["code"] != "Ok"
    )
    writer.writerow(
        [
            index,
            f"{start[1]:.7f}",
            f"{start[0]:.7f}",
            f"{target[1]:.7f}",
            f"{target[0]:.7f}",
            departure,
            seconds_to_minutes(departure),
            static["code"],
            depart["code"],
            asymmetric["code"],
            asymmetric_optimized["code"],
            overlay["code"],
            static["duration"],
            seconds_to_minutes(static["duration"]),
            depart["duration"],
            seconds_to_minutes(depart["duration"]),
            asymmetric["duration"],
            seconds_to_minutes(asymmetric["duration"]),
            asymmetric_optimized["duration"],
            seconds_to_minutes(asymmetric_optimized["duration"]),
            overlay["duration"],
            seconds_to_minutes(overlay["duration"]),
            int(depart_effect),
            int(asym_effect),
            int(asym_optimized_effect),
            int(overlay_effect),
            int(static_ok_asym_failed),
            int(static_ok_asym_optimized_failed),
            int(static_ok_overlay_failed),
            int(asymmetric_ok_overlay_failed),
            int(asymmetric_optimized_ok_overlay_failed),
            asymmetric["message"],
            asymmetric_optimized["message"],
            overlay["message"],
            f"{static['elapsed_ms']:.2f}",
            f"{depart['elapsed_ms']:.2f}",
            f"{asymmetric['elapsed_ms']:.2f}",
            f"{asymmetric_optimized['elapsed_ms']:.2f}",
            f"{overlay['elapsed_ms']:.2f}",
        ]
    )


def process_pair(args, index, start, target, departure):
    if args.snap_samples:
        snapped_start = snap(args.host, start, args.timeout, args.workers)
        snapped_target = snap(args.host, target, args.timeout, args.workers)
        if snapped_start is None or snapped_target is None:
            return {
                "index": index,
                "snap_failed": True,
                "start": start,
                "target": target,
            }
        start, target = snapped_start, snapped_target

    static = route(args.host, start, target, args.timeout, args.workers)
    depart = route(args.host, start, target, args.timeout, args.workers, departure)
    asymmetric = route(
        args.host,
        start,
        target,
        args.timeout,
        args.workers,
        departure,
        mode="asymmetric",
    )
    asymmetric_optimized = route(
        args.host,
        start,
        target,
        args.timeout,
        args.workers,
        departure,
        mode="asymmetric_optimized",
    )
    overlay = route(
        args.host,
        start,
        target,
        args.timeout,
        args.workers,
        departure,
        mode="overlay_asymmetric",
    )

    depart_effect = differs(static, depart, args.duration_tolerance)
    asym_effect = differs(static, asymmetric, args.duration_tolerance)
    asym_optimized_effect = differs(static, asymmetric_optimized, args.duration_tolerance)
    overlay_effect = differs(static, overlay, args.duration_tolerance)

    return {
        "index": index,
        "snap_failed": False,
        "start": start,
        "target": target,
        "departure": departure,
        "static": static,
        "depart": depart,
        "asymmetric": asymmetric,
        "asymmetric_optimized": asymmetric_optimized,
        "overlay": overlay,
        "depart_effect": depart_effect,
        "asymmetric_effect": asym_effect,
        "asymmetric_optimized_effect": asym_optimized_effect,
        "overlay_effect": overlay_effect,
    }


def run(args):
    random.seed(args.seed)

    points = read_points(args.points) if args.points else []
    output_handle = open(args.output_csv, "w", newline="", encoding="utf-8") if args.output_csv else None
    writer = csv.writer(output_handle) if output_handle else None
    if writer:
        write_csv_header(writer)

    counters = {
        "sampled": 0,
        "static_ok": 0,
        "depart_ok": 0,
        "asymmetric_ok": 0,
        "asymmetric_optimized_ok": 0,
        "overlay_ok": 0,
        "depart_effect": 0,
        "asymmetric_effect": 0,
        "asymmetric_optimized_effect": 0,
        "overlay_effect": 0,
        "no_temporal_effect": 0,
        "static_ok_asymmetric_failed": 0,
        "static_ok_asymmetric_optimized_failed": 0,
        "static_ok_overlay_failed": 0,
        "asymmetric_ok_overlay_failed": 0,
        "asymmetric_optimized_ok_overlay_failed": 0,
        "snap_failed": 0,
    }
    latencies = {
        "static": [],
        "depart": [],
        "asymmetric": [],
        "asymmetric_optimized": [],
        "overlay": [],
    }
    ok_latencies = {
        "static": [],
        "depart": [],
        "asymmetric": [],
        "asymmetric_optimized": [],
        "overlay": [],
    }

    try:
        work_items = [
            (
                index,
                *sample_pair(points, args.bbox, args.max_straight_line_km),
                random.randint(*args.departure_range)
                if args.departure_range
                else args.departure,
            )
            for index in range(args.requests)
        ]

        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(process_pair, args, index, start, target, departure): index
                for index, start, target, departure in work_items
            }

            for completed, future in enumerate(as_completed(futures), start=1):
                try:
                    result = future.result()
                except Exception as error:
                    print(f"Worker error for request {futures[future]}: {error}")
                    continue

                if result["snap_failed"]:
                    counters["snap_failed"] += 1
                    continue

                index = result["index"]
                start = result["start"]
                target = result["target"]
                departure = result["departure"]
                static = result["static"]
                depart = result["depart"]
                asymmetric = result["asymmetric"]
                asymmetric_optimized = result["asymmetric_optimized"]
                overlay = result["overlay"]
                depart_effect = result["depart_effect"]
                asym_effect = result["asymmetric_effect"]
                asym_optimized_effect = result["asymmetric_optimized_effect"]
                overlay_effect = result["overlay_effect"]

                counters["sampled"] += 1
                counters["static_ok"] += int(static["code"] == "Ok")
                counters["depart_ok"] += int(depart["code"] == "Ok")
                counters["asymmetric_ok"] += int(asymmetric["code"] == "Ok")
                counters["asymmetric_optimized_ok"] += int(
                    asymmetric_optimized["code"] == "Ok"
                )
                counters["overlay_ok"] += int(overlay["code"] == "Ok")
                counters["depart_effect"] += int(depart_effect)
                counters["asymmetric_effect"] += int(asym_effect)
                counters["asymmetric_optimized_effect"] += int(asym_optimized_effect)
                counters["overlay_effect"] += int(overlay_effect)
                counters["no_temporal_effect"] += int(
                    static["code"] == "Ok"
                    and depart["code"] == "Ok"
                    and asymmetric["code"] == "Ok"
                    and asymmetric_optimized["code"] == "Ok"
                    and overlay["code"] == "Ok"
                    and not depart_effect
                    and not asym_effect
                    and not asym_optimized_effect
                    and not overlay_effect
                )
                counters["static_ok_asymmetric_failed"] += int(
                    static["code"] == "Ok" and asymmetric["code"] != "Ok"
                )
                counters["static_ok_asymmetric_optimized_failed"] += int(
                    static["code"] == "Ok" and asymmetric_optimized["code"] != "Ok"
                )
                counters["static_ok_overlay_failed"] += int(
                    static["code"] == "Ok" and overlay["code"] != "Ok"
                )
                counters["asymmetric_ok_overlay_failed"] += int(
                    asymmetric["code"] == "Ok" and overlay["code"] != "Ok"
                )
                counters["asymmetric_optimized_ok_overlay_failed"] += int(
                    asymmetric_optimized["code"] == "Ok" and overlay["code"] != "Ok"
                )
                latencies["static"].append(static["elapsed_ms"])
                latencies["depart"].append(depart["elapsed_ms"])
                latencies["asymmetric"].append(asymmetric["elapsed_ms"])
                latencies["asymmetric_optimized"].append(asymmetric_optimized["elapsed_ms"])
                latencies["overlay"].append(overlay["elapsed_ms"])
                if static["code"] == "Ok":
                    ok_latencies["static"].append(static["elapsed_ms"])
                if depart["code"] == "Ok":
                    ok_latencies["depart"].append(depart["elapsed_ms"])
                if asymmetric["code"] == "Ok":
                    ok_latencies["asymmetric"].append(asymmetric["elapsed_ms"])
                if asymmetric_optimized["code"] == "Ok":
                    ok_latencies["asymmetric_optimized"].append(
                        asymmetric_optimized["elapsed_ms"]
                    )
                if overlay["code"] == "Ok":
                    ok_latencies["overlay"].append(overlay["elapsed_ms"])

                if writer:
                    write_csv_row(
                        writer,
                        index,
                        start,
                        target,
                        departure,
                        static,
                        depart,
                        asymmetric,
                        asymmetric_optimized,
                        overlay,
                        depart_effect,
                        asym_effect,
                        asym_optimized_effect,
                        overlay_effect,
                    )

                if args.progress_every and completed % args.progress_every == 0:
                    print(
                        f"{completed}/{args.requests}: "
                        f"static_ok={counters['static_ok']} "
                        f"asym_ok={counters['asymmetric_ok']} "
                        f"asym_opt_ok={counters['asymmetric_optimized_ok']} "
                        f"overlay_ok={counters['overlay_ok']} "
                        f"depart_effect={counters['depart_effect']} "
                        f"asym_effect={counters['asymmetric_effect']} "
                        f"asym_opt_effect={counters['asymmetric_optimized_effect']} "
                        f"overlay_effect={counters['overlay_effect']} "
                        f"asym_regressions={counters['static_ok_asymmetric_failed']}"
                        f" asym_opt_regressions={counters['static_ok_asymmetric_optimized_failed']}"
                        f" overlay_regressions={counters['static_ok_overlay_failed']}"
                        f" overlay_vs_asym={counters['asymmetric_ok_overlay_failed']}"
                        f" overlay_vs_asym_opt={counters['asymmetric_optimized_ok_overlay_failed']}"
                    )
    finally:
        if output_handle:
            output_handle.close()

    print("\nSummary")
    for key, value in counters.items():
        print(f"  {key}: {value}")

    print("\nLatency (ms, all samples)")
    for mode in ("static", "depart", "asymmetric", "asymmetric_optimized", "overlay"):
        stats = compute_latency_stats(latencies[mode])
        if stats is None:
            print(f"  {mode}: no samples")
            continue

        print(
            f"  {mode}: count={stats['count']} "
            f"mean={stats['mean']:.2f} "
            f"median={stats['median']:.2f} "
            f"p95={stats['p95']:.2f}"
        )

    print("\nLatency (ms, Ok only)")
    for mode in ("static", "depart", "asymmetric", "asymmetric_optimized", "overlay"):
        stats = compute_latency_stats(ok_latencies[mode])
        if stats is None:
            print(f"  {mode}: no Ok samples")
            continue

        print(
            f"  {mode}: count={stats['count']} "
            f"mean={stats['mean']:.2f} "
            f"median={stats['median']:.2f} "
            f"p95={stats['p95']:.2f}"
        )

    failed = False
    if args.fail_on_asymmetric_noroute and counters["static_ok_asymmetric_failed"] > 0:
        print(
            "\nFAIL: at least one pair routed statically but failed in temporal_mode=asymmetric"
        )
        failed = True

    if (
        args.fail_on_asymmetric_optimized_noroute
        and counters["static_ok_asymmetric_optimized_failed"] > 0
    ):
        print(
            "\nFAIL: at least one pair routed statically but failed in "
            "temporal_mode=asymmetric_optimized"
        )
        failed = True

    if args.fail_on_overlay_noroute and counters["static_ok_overlay_failed"] > 0:
        print(
            "\nFAIL: at least one pair routed statically but failed in "
            "temporal_mode=overlay_asymmetric"
        )
        failed = True

    if (
        args.fail_on_overlay_vs_asymmetric_noroute
        and counters["asymmetric_ok_overlay_failed"] > 0
    ):
        print(
            "\nFAIL: at least one pair routed in temporal_mode=asymmetric but failed in "
            "temporal_mode=overlay_asymmetric"
        )
        failed = True

    if (
        args.fail_on_overlay_vs_asymmetric_optimized_noroute
        and counters["asymmetric_optimized_ok_overlay_failed"] > 0
    ):
        print(
            "\nFAIL: at least one pair routed in temporal_mode=asymmetric_optimized but failed "
            "in temporal_mode=overlay_asymmetric"
        )
        failed = True

    total_temporal_effects = (
        counters["depart_effect"]
        + counters["asymmetric_effect"]
        + counters["asymmetric_optimized_effect"]
        + counters["overlay_effect"]
    )
    if total_temporal_effects < args.min_temporal_effects:
        print(
            "\nFAIL: temporal effect count "
            f"{total_temporal_effects} "
            f"is below --min-temporal-effects={args.min_temporal_effects}"
        )
        failed = True

    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="http://127.0.0.1:5000")
    parser.add_argument("--requests", type=int, default=100)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--departure", type=int, default=1735689600)
    parser.add_argument(
        "--departure-range",
        type=parse_departure_range,
        help="Randomize depart_at per request within min_timestamp,max_timestamp inclusive.",
    )
    parser.add_argument("--bbox", type=parse_bbox, default=DEFAULT_THESSALONIKI_BBOX)
    parser.add_argument(
        "--points",
        help="Optional CSV/TSV file with lon/lat or @lon/@lat columns. If omitted, random bbox points are used.",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--duration-tolerance",
        type=float,
        default=0.1,
        help="Seconds of route-duration difference needed to count as a temporal effect.",
    )
    parser.add_argument(
        "--max-straight-line-km",
        type=float,
        default=4.0,
        help="Maximum straight-line distance for generated pairs. Use 0 to disable.",
    )
    parser.add_argument(
        "--snap-samples",
        action="store_true",
        help="Call /nearest for each random point before routing.",
    )
    parser.add_argument("--output-csv", help="Optional per-request result CSV path.")
    parser.add_argument("--progress-every", type=int, default=10)
    parser.add_argument(
        "--min-temporal-effects",
        type=int,
        default=0,
        help="Fail unless at least this many depart/asymmetric/asymmetric_optimized/overlay requests differ from static.",
    )
    parser.add_argument(
        "--fail-on-asymmetric-noroute",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Fail if static route is Ok but temporal_mode=asymmetric is not Ok.",
    )
    parser.add_argument(
        "--fail-on-asymmetric-optimized-noroute",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Fail if static route is Ok but temporal_mode=asymmetric_optimized is not Ok.",
    )
    parser.add_argument(
        "--fail-on-overlay-noroute",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Fail if static route is Ok but temporal_mode=overlay_asymmetric is not Ok.",
    )
    parser.add_argument(
        "--fail-on-overlay-vs-asymmetric-noroute",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Fail if temporal_mode=asymmetric is Ok but temporal_mode=overlay_asymmetric is not Ok.",
    )
    parser.add_argument(
        "--fail-on-overlay-vs-asymmetric-optimized-noroute",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Fail if temporal_mode=asymmetric_optimized is Ok but temporal_mode=overlay_asymmetric is not Ok.",
    )

    args = parser.parse_args()
    if args.requests <= 0:
        parser.error("--requests must be positive")
    if args.workers <= 0:
        parser.error("--workers must be positive")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
