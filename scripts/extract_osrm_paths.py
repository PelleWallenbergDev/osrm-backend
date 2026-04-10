#!/usr/bin/env python3
"""
Extract paths from OSRM for coordinate pairs and save to CSV.

For each pair in the input CSV, requests a route from OSRM,
extracts the path (geometry), and saves it to an output CSV file.
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
        status, text = session.get(url, headers={"User-Agent": "osrm-extract-paths"})
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        try:
            payload = json.loads(text)
        except ValueError:
            payload = {"code": "HTTPError", "message": text}
        return status, payload, elapsed_ms
    except (OSError, http.client.HTTPException, ValueError) as error:
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return 0, {"code": "TransportError", "message": str(error)}, elapsed_ms


def route_url(host, start, target, departure=None):
    coordinates = f"{start[0]:.7f},{start[1]:.7f};{target[0]:.7f},{target[1]:.7f}"
    params = {
        "overview": "full",
        "annotations": "duration,distance",
    }
    if departure is not None:
        params["depart_at"] = str(departure)

    return f"{host.rstrip('/')}/route/v1/driving/{coordinates}?{urllib.parse.urlencode(params)}"


def route(host, start, target, timeout, pool_size, departure=None):
    status, payload, elapsed_ms = open_json(
        route_url(host, start, target, departure), timeout, pool_size
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
        result["geometry"] = best.get("geometry")  # Encoded polyline

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
            try:
                departure = parse_departure(departure_str, base_date)
            except ValueError as e:
                print(f"Error parsing departure for row {index}: {e}, using default")
                departure = 1735689600  # default
            pairs.append((index, start, target, departure))
        return pairs


def write_csv_header(writer):
    writer.writerow(
        [
            "index",
            "start_lat",
            "start_lon",
            "target_lat",
            "target_lon",
            "duration_seconds",
            "distance_meters",
            "code",
            "geometry",
        ]
    )


def write_csv_row(writer, index, start, target, route_result):
    writer.writerow(
        [
            index,
            f"{start[1]:.7f}",
            f"{start[0]:.7f}",
            f"{target[1]:.7f}",
            f"{target[0]:.7f}",
            route_result["duration"],
            route_result["distance"],
            route_result["code"],
            route_result["geometry"] if route_result["geometry"] else "",
        ]
    )


def process_pair(args, index, start, target, departure):
    route_result = route(args.host, start, target, args.timeout, args.workers, departure)

    return {
        "index": index,
        "start": start,
        "target": target,
        "route": route_result,
    }


def run(args):
    pairs = read_pairs(args.csv)
    work_items = pairs

    output_handle = open(args.output_csv, "w", newline="", encoding="utf-8") if args.output_csv else None
    writer = csv.writer(output_handle) if output_handle else None
    if writer:
        write_csv_header(writer)

    counters = {
        "processed": 0,
        "success": 0,
        "failed": 0,
    }

    try:
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
                    counters["failed"] += 1
                    continue

                index = result["index"]
                start = result["start"]
                target = result["target"]
                route_result = result["route"]

                counters["processed"] += 1
                if route_result["code"] == "Ok":
                    counters["success"] += 1
                else:
                    counters["failed"] += 1

                if writer:
                    write_csv_row(writer, index, start, target, route_result)

                if completed % 10 == 0 or completed == len(work_items):
                    print(f"Completed {completed}/{len(work_items)} requests")

    finally:
        if output_handle:
            output_handle.close()

    print("\nSummary")
    for key, value in counters.items():
        print(f"  {key}: {value}")

    return 0 if counters["failed"] == 0 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="http://127.0.0.1:5000")
    parser.add_argument("--csv", required=True, help="CSV file with coordinate pairs")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output-csv", required=True, help="Output CSV file for paths")

    args = parser.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
