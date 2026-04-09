#!/usr/bin/env python3
"""
Smoke-test temporal OSRM route behavior over many coordinate pairs.

The script compares three requests for each sampled pair:

  1. static route
  2. depart_at route, which temporalizes durations on the selected route
  3. depart_at + temporal_mode=asymmetric, which uses the experimental
     asymmetric temporal search path

It classifies pairs as temporal-effect or no-temporal-effect by comparing
duration and geometry against the static baseline, and it flags the important
regression where static routing succeeds but asymmetric temporal routing returns
NoRoute.
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
    if mode == "asymmetric":
        params["temporal_mode"] = "asymmetric"

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
            "start_lon",
            "start_lat",
            "target_lon",
            "target_lat",
            "departure",
            "static_code",
            "depart_code",
            "asymmetric_code",
            "static_duration",
            "depart_duration",
            "asymmetric_duration",
            "depart_effect",
            "asymmetric_effect",
            "static_ok_asymmetric_failed",
            "asymmetric_message",
            "static_ms",
            "depart_ms",
            "asymmetric_ms",
        ]
    )


def write_csv_row(
    writer, index, start, target, departure, static, depart, asymmetric, depart_effect, asym_effect
):
    static_ok_asym_failed = static["code"] == "Ok" and asymmetric["code"] != "Ok"
    writer.writerow(
        [
            index,
            f"{start[0]:.7f}",
            f"{start[1]:.7f}",
            f"{target[0]:.7f}",
            f"{target[1]:.7f}",
            departure,
            static["code"],
            depart["code"],
            asymmetric["code"],
            static["duration"],
            depart["duration"],
            asymmetric["duration"],
            int(depart_effect),
            int(asym_effect),
            int(static_ok_asym_failed),
            asymmetric["message"],
            f"{static['elapsed_ms']:.2f}",
            f"{depart['elapsed_ms']:.2f}",
            f"{asymmetric['elapsed_ms']:.2f}",
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

    depart_effect = differs(static, depart, args.duration_tolerance)
    asym_effect = differs(static, asymmetric, args.duration_tolerance)

    return {
        "index": index,
        "snap_failed": False,
        "start": start,
        "target": target,
        "departure": departure,
        "static": static,
        "depart": depart,
        "asymmetric": asymmetric,
        "depart_effect": depart_effect,
        "asymmetric_effect": asym_effect,
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
        "depart_effect": 0,
        "asymmetric_effect": 0,
        "no_temporal_effect": 0,
        "static_ok_asymmetric_failed": 0,
        "snap_failed": 0,
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
                depart_effect = result["depart_effect"]
                asym_effect = result["asymmetric_effect"]

                counters["sampled"] += 1
                counters["static_ok"] += int(static["code"] == "Ok")
                counters["depart_ok"] += int(depart["code"] == "Ok")
                counters["asymmetric_ok"] += int(asymmetric["code"] == "Ok")
                counters["depart_effect"] += int(depart_effect)
                counters["asymmetric_effect"] += int(asym_effect)
                counters["no_temporal_effect"] += int(
                    static["code"] == "Ok"
                    and depart["code"] == "Ok"
                    and asymmetric["code"] == "Ok"
                    and not depart_effect
                    and not asym_effect
                )
                counters["static_ok_asymmetric_failed"] += int(
                    static["code"] == "Ok" and asymmetric["code"] != "Ok"
                )

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
                        depart_effect,
                        asym_effect,
                    )

                if args.progress_every and completed % args.progress_every == 0:
                    print(
                        f"{completed}/{args.requests}: "
                        f"static_ok={counters['static_ok']} "
                        f"asym_ok={counters['asymmetric_ok']} "
                        f"depart_effect={counters['depart_effect']} "
                        f"asym_effect={counters['asymmetric_effect']} "
                        f"asym_regressions={counters['static_ok_asymmetric_failed']}"
                    )
    finally:
        if output_handle:
            output_handle.close()

    print("\nSummary")
    for key, value in counters.items():
        print(f"  {key}: {value}")

    failed = False
    if args.fail_on_asymmetric_noroute and counters["static_ok_asymmetric_failed"] > 0:
        print(
            "\nFAIL: at least one pair routed statically but failed in temporal_mode=asymmetric"
        )
        failed = True

    if counters["depart_effect"] + counters["asymmetric_effect"] < args.min_temporal_effects:
        print(
            "\nFAIL: temporal effect count "
            f"{counters['depart_effect'] + counters['asymmetric_effect']} "
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
        help="Fail unless at least this many depart/asymmetric requests differ from static.",
    )
    parser.add_argument(
        "--fail-on-asymmetric-noroute",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Fail if static route is Ok but temporal_mode=asymmetric is not Ok.",
    )

    args = parser.parse_args()
    if args.requests <= 0:
        parser.error("--requests must be positive")
    if args.workers <= 0:
        parser.error("--workers must be positive")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
