#!/usr/bin/env python3
"""
Test temporal OSRM route behavior using coordinate pairs from a CSV file.

The script compares three requests for each pair in the CSV:

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
import sys
import threading
import time
import urllib.parse
from datetime import datetime, timedelta


THREAD_LOCAL = threading.local()
TEMPORAL_DEBUG_FIELDS = [
    "endpoint_pairs_tried",
    "endpoint_pairs_with_static_upper_bound",
    "static_upper_bound_pair_match_count",
    "static_upper_bound_source_mismatch_count",
    "static_upper_bound_target_mismatch_count",
    "static_upper_bound_direction_mismatch_count",
    "static_upper_bound_route_endpoint_unavailable_count",
    "first_static_upper_bound_mismatch_directed_source_node",
    "first_static_upper_bound_mismatch_directed_target_node",
    "first_static_upper_bound_mismatch_route_source_node",
    "first_static_upper_bound_mismatch_route_target_node",
    "reverse_lower_bound_source_invalid",
    "queue_exhausted_without_target",
    "pruned_by_initial_upper_bound",
    "pruned_by_best_upper_bound",
    "invalid_duration_relaxations",
    "target_reached",
    "best_candidate_rejected",
    "expanded_nodes",
    "relaxation_attempts",
    "relaxation_improvements",
    "source_first_pop_pruned_by_initial_upper_bound",
    "min_initial_upper_bound_prune_margin",
    "max_initial_upper_bound_prune_margin",
]

day_map = {
    "mon": 0,
    "tue": 1,
    "wed": 2,
    "tor": 3,  # torsdag
    "thurs": 3,
    "fri": 4,
    "sat": 5,
    "sun": 6,
    "sön": 6,
}


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


def parse_departure(departure_str, base_date):
    parts = departure_str.split()
    if len(parts) >= 2:
        day_str = parts[0].lower()
        time_str = ' '.join(parts[1:])
    else:
        raise ValueError(f"Invalid departure format: {departure_str}")
    
    day_num = day_map.get(day_str)
    if day_num is None:
        raise ValueError(f"Unknown day {day_str}")
    
    # Parse time
    try:
        dt_time = datetime.strptime(time_str, "%I:%M:%S %p").time()
    except ValueError:
        try:
            dt_time = datetime.strptime(time_str, "%H:%M").time()
        except ValueError:
            try:
                dt_time = datetime.strptime(time_str, "%I:%M %p").time()
            except ValueError:
                raise ValueError(f"Invalid time format: {time_str}")
    
    # Find next occurrence
    current_weekday = base_date.weekday()
    days_ahead = (day_num - current_weekday) % 7
    if days_ahead == 0 and dt_time <= base_date.time():
        days_ahead = 7
    next_date = base_date + timedelta(days=days_ahead)
    departure_dt = datetime.combine(next_date, dt_time)
    return int(departure_dt.timestamp())


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
        status, text = session.get(url, headers={"User-Agent": "osrm-temporal-csv"})
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        try:
            payload = json.loads(text)
        except ValueError:
            payload = {"code": "HTTPError", "message": text}
        return status, payload, elapsed_ms
    except (OSError, http.client.HTTPException, ValueError) as error:
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return 0, {"code": "TransportError", "message": str(error)}, elapsed_ms


def route_url(host, start, target, departure, mode, temporal_debug):
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
        if temporal_debug:
            params["temporal_debug"] = "true"

    return f"{host.rstrip('/')}/route/v1/driving/{coordinates}?{urllib.parse.urlencode(params)}"


def nearest_url(host, coord):
    params = urllib.parse.urlencode({"number": "1"})
    return f"{host.rstrip('/')}/nearest/v1/driving/{coord[0]:.7f},{coord[1]:.7f}?{params}"


def route(
    host, start, target, timeout, pool_size, departure=None, mode="static", temporal_debug=False
):
    status, payload, elapsed_ms = open_json(
        route_url(host, start, target, departure, mode, temporal_debug), timeout, pool_size
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
        "temporal_debug": payload.get("temporal_debug", {}),
    }

    if code == "Ok" and payload.get("routes"):
        best = payload["routes"][0]
        result["duration"] = float(best.get("duration", 0.0))
        result["distance"] = float(best.get("distance", 0.0))
        result["geometry"] = best.get("geometry")

    return result


def read_pairs(path):
    base_date = datetime(2026, 4, 10)
    with open(path, newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        pairs = []
        for row in reader:
            index = int(row['Index'])
            start = (float(row['SourceLong']), float(row['SourceLat']))
            target = (float(row['TargetLong']), float(row['TargetLat']))
            departure_str = row['Depature time']
            duration_google_min = row['Duration Google (min)']
            try:
                departure = parse_departure(departure_str, base_date)
            except ValueError as e:
                print(f"Error parsing departure for row {index}: {e}, using default")
                departure = 1735689600  # default
            pairs.append((index, start, target, departure, duration_google_min))
        return pairs


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
            "departure_minutes",
            "static_code",
            "depart_code",
            "asymmetric_code",
            "static_duration_minutes",
            "depart_duration_minutes",
            "asymmetric_duration_minutes",
            "duration_google_min",
        ]
    )


def seconds_to_minutes(value):
    if value is None:
        return None
    return value / 60.0


def write_csv_row(
    writer, index, start, target, departure, static, depart, asymmetric, duration_google_min
):
    writer.writerow(
        [
            index,
            f"{start[1]:.7f}",
            f"{start[0]:.7f}",
            f"{target[1]:.7f}",
            f"{target[0]:.7f}",
            seconds_to_minutes(departure),
            static["code"],
            depart["code"],
            asymmetric["code"],
            seconds_to_minutes(static["duration"]),
            seconds_to_minutes(depart["duration"]),
            seconds_to_minutes(asymmetric["duration"]),
            duration_google_min,
        ]
    )


def process_pair(args, index, start, target, departure, duration_google_min):
    static = route(args.host, start, target, args.timeout, args.workers)
    depart = route(args.host, start, target, args.timeout, args.workers, departure)
    asymmetric = route(
        args.host,
        start, target,
        args.timeout,
        args.workers,
        departure,
        mode="asymmetric",
        temporal_debug=args.temporal_debug,
    )

    depart_effect = differs(static, depart, args.duration_tolerance)
    asym_effect = differs(static, asymmetric, args.duration_tolerance)

    return {
        "index": index,
        "start": start,
        "target": target,
        "departure": departure,
        "static": static,
        "depart": depart,
        "asymmetric": asymmetric,
        "depart_effect": depart_effect,
        "asymmetric_effect": asym_effect,
        "duration_google_min": duration_google_min,
    }


def run(args):
    pairs = read_pairs(args.csv)
    work_items = pairs

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
    }

    try:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(process_pair, args, index, start, target, departure, duration_google_min): index
                for index, start, target, departure, duration_google_min in work_items
            }

            for completed, future in enumerate(as_completed(futures), start=1):
                try:
                    result = future.result()
                except Exception as error:
                    print(f"Worker error for request {futures[future]}: {error}")
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
                duration_google_min = result["duration_google_min"]

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
                        writer, index, start, target, departure, static, depart, asymmetric, duration_google_min
                    )

                if completed % 10 == 0 or completed == len(work_items):
                    print(f"Completed {completed}/{len(work_items)} requests")

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
    parser.add_argument("--csv", required=True, help="CSV file with coordinate pairs")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--duration-tolerance", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output-csv", help="Output CSV file for results")
    parser.add_argument("--temporal-debug", action="store_true", help="Enable temporal debug output")
    parser.add_argument("--fail-on-asymmetric-noroute", action="store_true")
    parser.add_argument("--min-temporal-effects", type=int, default=0)

    args = parser.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()