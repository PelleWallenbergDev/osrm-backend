#!/usr/bin/env python3
"""
Query one route across every temporal week bucket and graph the travel duration.

The script can evaluate one or more route modes:

  - static
  - depart
  - asymmetric
  - overlay_asymmetric

It writes a wide CSV with one row per week bucket and produces a self-contained
SVG graph with:

  - x axis: week bucket
  - y axis: travel duration

The bucket semantics match OSRM's temporal bucket logic:

  - Monday-based week
  - UTC time
"""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import datetime as dt
import http.client
import json
import math
import sys
import threading
import urllib.parse


DEFAULT_WEEK_START_TIMESTAMP = 1735516800  # 2024-12-30 00:00:00 UTC, Monday
DEFAULT_BUCKET_SIZE_MINUTES = 15
DEFAULT_WEEK_BUCKET_COUNT = 672
DEFAULT_MODES = ["overlay_asymmetric"]
MODE_COLORS = {
    "static": "#1f77b4",
    "depart": "#ff7f0e",
    "asymmetric": "#2ca02c",
    "overlay_asymmetric": "#d62728",
}
MODE_LABELS = {
    "static": "static",
    "depart": "depart",
    "asymmetric": "asymmetric",
    "overlay_asymmetric": "overlay_asymmetric",
}
THREAD_LOCAL = threading.local()


class ThreadLocalHTTPSession:
    def __init__(self, timeout):
        self.timeout = timeout
        self.connections = {}

    def close(self):
        for connection in self.connections.values():
            connection.close()
        self.connections.clear()

    def _get_connection(self, parsed):
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

        key, connection = self._get_connection(parsed)
        try:
            connection.request("GET", path, headers=headers)
            response = connection.getresponse()
            body = response.read().decode("utf-8", errors="replace")
            return response.status, body
        except (OSError, http.client.HTTPException):
            self.connections.pop(key, None)
            try:
                connection.close()
            except OSError:
                pass

            key, connection = self._get_connection(parsed)
            connection.request("GET", path, headers=headers)
            response = connection.getresponse()
            body = response.read().decode("utf-8", errors="replace")
            return response.status, body


def get_session(timeout):
    session = getattr(THREAD_LOCAL, "session", None)
    if session is None:
        session = ThreadLocalHTTPSession(timeout)
        THREAD_LOCAL.session = session
    return session


def parse_coord(value):
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("coordinate must be lon,lat")
    try:
        lon = float(parts[0])
        lat = float(parts[1])
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid coordinate {value!r}: {error}") from error
    return lon, lat


def parse_modes(value):
    modes = [part.strip() for part in value.split(",") if part.strip()]
    valid = {"static", "depart", "asymmetric", "overlay_asymmetric"}
    if not modes:
        raise argparse.ArgumentTypeError("at least one mode is required")
    invalid = [mode for mode in modes if mode not in valid]
    if invalid:
        raise argparse.ArgumentTypeError(
            "invalid mode(s): " + ", ".join(invalid) + ". "
            "Valid modes: static, depart, asymmetric, overlay_asymmetric"
        )
    deduped = []
    for mode in modes:
        if mode not in deduped:
            deduped.append(mode)
    return deduped


def parse_bucket_range(value):
    parts = [part.strip() for part in value.split(":")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("bucket range must be start:end")
    try:
        start = int(parts[0])
        end = int(parts[1])
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid bucket range {value!r}: {error}") from error
    if start < 0 or end < start:
        raise argparse.ArgumentTypeError("bucket range must satisfy 0 <= start <= end")
    return start, end


def escape_xml(value):
    return (
        str(value)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def bucket_timestamp(week_start_timestamp, bucket, bucket_size_minutes):
    return week_start_timestamp + bucket * bucket_size_minutes * 60


def bucket_label(timestamp):
    instant = dt.datetime.fromtimestamp(timestamp, tz=dt.timezone.utc)
    return instant.strftime("%a %H:%M")


def route_url(host, start, target, departure_timestamp, mode):
    coordinates = f"{start[0]:.7f},{start[1]:.7f};{target[0]:.7f},{target[1]:.7f}"
    params = {
        "overview": "false",
        "steps": "false",
        "annotations": "false",
        "alternatives": "false",
    }

    if mode != "static":
        params["depart_at"] = str(departure_timestamp)

    if mode in {"asymmetric", "overlay_asymmetric"}:
        params["temporal_mode"] = mode

    encoded = urllib.parse.urlencode(params)
    return f"{host.rstrip('/')}/route/v1/driving/{coordinates}?{encoded}"


def nearest_url(host, coord):
    encoded = urllib.parse.urlencode({"number": "1"})
    return f"{host.rstrip('/')}/nearest/v1/driving/{coord[0]:.7f},{coord[1]:.7f}?{encoded}"


def open_json(url, user_agent, timeout):
    session = get_session(timeout)
    try:
        status, text = session.get(url, headers={"User-Agent": user_agent})
    except (OSError, http.client.HTTPException, ValueError) as error:
        return 0, {"code": "TransportError", "message": str(error)}

    try:
        payload = json.loads(text)
    except ValueError:
        payload = {"code": "HTTPError", "message": text}
    return status, payload


def snap_coordinate(host, coord, timeout):
    status, payload = open_json(
        nearest_url(host, coord), "osrm-temporal-bucket-graph", timeout
    )
    if status != 200 or payload.get("code") != "Ok" or not payload.get("waypoints"):
        return None

    location = payload["waypoints"][0].get("location")
    if not location or len(location) != 2:
        return None
    return float(location[0]), float(location[1])


def route(host, start, target, departure_timestamp, mode, timeout):
    status, payload = open_json(
        route_url(host, start, target, departure_timestamp, mode),
        "osrm-temporal-bucket-graph",
        timeout,
    )
    code = payload.get("code", "MissingCode")
    result = {
        "http_status": status,
        "code": code,
        "message": payload.get("message", ""),
        "duration_seconds": None,
        "distance_meters": None,
    }
    if code == "Ok" and payload.get("routes"):
        best = payload["routes"][0]
        result["duration_seconds"] = float(best.get("duration", 0.0))
        result["distance_meters"] = float(best.get("distance", 0.0))
    return result


def write_csv(path, rows, modes):
    fieldnames = [
        "bucket",
        "bucket_label_utc",
        "departure_timestamp",
        "departure_iso_utc",
    ]
    for mode in modes:
        prefix = mode
        fieldnames.extend(
            [
                f"{prefix}_code",
                f"{prefix}_duration_seconds",
                f"{prefix}_duration_minutes",
                f"{prefix}_distance_meters",
                f"{prefix}_message",
            ]
        )

    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_svg(rows, modes, output_path, pair_label, y_unit, bucket_size_minutes):
    successful_values = []
    for row in rows:
        for mode in modes:
            duration_seconds = row.get(f"{mode}_duration_seconds")
            if duration_seconds is not None:
                successful_values.append(float(duration_seconds))

    if not successful_values:
        y_min = 0.0
        y_max = 1.0
    else:
        y_min = min(successful_values)
        y_max = max(successful_values)
        if math.isclose(y_min, y_max):
            y_min = 0.0
            y_max = y_max * 1.1 if y_max > 0 else 1.0
        else:
            padding = (y_max - y_min) * 0.08
            y_min = max(0.0, y_min - padding)
            y_max = y_max + padding

    if y_unit == "minutes":
        y_scale = 1.0 / 60.0
        y_title = "Travel Duration (minutes)"
    else:
        y_scale = 1.0
        y_title = "Travel Duration (seconds)"

    width = 1400
    height = 840
    margin_left = 90
    margin_right = 30
    margin_top = 70
    margin_bottom = 130
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    bucket_min = rows[0]["bucket"]
    bucket_max = rows[-1]["bucket"] if rows else 1
    bucket_span = max(1, bucket_max - bucket_min)

    def x_for_bucket(bucket):
        return margin_left + plot_width * (bucket - bucket_min) / bucket_span

    def y_for_duration(duration_seconds):
        scaled = duration_seconds * y_scale
        scaled_min = y_min * y_scale
        scaled_max = y_max * y_scale
        span = max(1e-9, scaled_max - scaled_min)
        return margin_top + plot_height * (1.0 - (scaled - scaled_min) / span)

    svg = []
    svg.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">'
    )
    svg.append(
        '<style>'
        'text { font-family: "Segoe UI", "Helvetica Neue", Arial, sans-serif; fill: #1f2933; }'
        '.title { font-size: 24px; font-weight: 600; }'
        '.subtitle { font-size: 14px; fill: #52606d; }'
        '.axis { stroke: #364152; stroke-width: 1.3; }'
        '.grid { stroke: #d9e2ec; stroke-width: 1; }'
        '.legend-label { font-size: 13px; }'
        '.tick { font-size: 12px; fill: #52606d; }'
        '.fail-label { font-size: 11px; fill: #9b1c1c; }'
        '</style>'
    )
    svg.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f8fbff"/>')
    svg.append(
        f'<text class="title" x="{margin_left}" y="36">Temporal Route Duration By Week Bucket</text>'
    )
    svg.append(
        f'<text class="subtitle" x="{margin_left}" y="58">{escape_xml(pair_label)}</text>'
    )

    scaled_y_min = y_min * y_scale
    scaled_y_max = y_max * y_scale
    for index in range(6):
        fraction = index / 5.0
        y = margin_top + plot_height * (1.0 - fraction)
        value = scaled_y_min + (scaled_y_max - scaled_y_min) * fraction
        svg.append(
            f'<line class="grid" x1="{margin_left}" y1="{y:.2f}" '
            f'x2="{margin_left + plot_width}" y2="{y:.2f}"/>'
        )
        svg.append(
            f'<text class="tick" x="{margin_left - 12}" y="{y + 4:.2f}" text-anchor="end">'
            f"{value:.1f}</text>"
        )

    bucket_size = max(1, rows[1]["bucket"] - rows[0]["bucket"]) if len(rows) > 1 else 1
    day_step = max(1, int(round((24 * 60) / max(1, bucket_size * bucket_size_minutes))))
    label_step = max(1, day_step)
    for index, row in enumerate(rows):
        if index % label_step != 0 and index != len(rows) - 1:
            continue
        x = x_for_bucket(row["bucket"])
        label = row["bucket_label_utc"]
        svg.append(
            f'<line class="grid" x1="{x:.2f}" y1="{margin_top}" '
            f'x2="{x:.2f}" y2="{margin_top + plot_height}"/>'
        )
        svg.append(
            f'<text class="tick" x="{x:.2f}" y="{margin_top + plot_height + 18}" '
            f'text-anchor="middle">{escape_xml(label)}</text>'
        )
        svg.append(
            f'<text class="tick" x="{x:.2f}" y="{margin_top + plot_height + 34}" '
            f'text-anchor="middle">b{row["bucket"]}</text>'
        )

    svg.append(
        f'<line class="axis" x1="{margin_left}" y1="{margin_top + plot_height}" '
        f'x2="{margin_left + plot_width}" y2="{margin_top + plot_height}"/>'
    )
    svg.append(
        f'<line class="axis" x1="{margin_left}" y1="{margin_top}" '
        f'x2="{margin_left}" y2="{margin_top + plot_height}"/>'
    )
    svg.append(
        f'<text x="{margin_left + plot_width / 2:.2f}" y="{height - 32}" '
        f'text-anchor="middle">Week Bucket (Monday-based UTC)</text>'
    )
    svg.append(
        f'<text x="24" y="{margin_top + plot_height / 2:.2f}" '
        f'transform="rotate(-90, 24, {margin_top + plot_height / 2:.2f})" '
        f'text-anchor="middle">{escape_xml(y_title)}</text>'
    )

    legend_x = margin_left
    legend_y = height - 82
    for index, mode in enumerate(modes):
        color = MODE_COLORS.get(mode, "#111111")
        x = legend_x + index * 220
        svg.append(
            f'<line x1="{x}" y1="{legend_y}" x2="{x + 28}" y2="{legend_y}" '
            f'stroke="{color}" stroke-width="3" stroke-linecap="round"/>'
        )
        svg.append(
            f'<text class="legend-label" x="{x + 38}" y="{legend_y + 5}">'
            f"{escape_xml(MODE_LABELS.get(mode, mode))}</text>"
        )

    for mode in modes:
        color = MODE_COLORS.get(mode, "#111111")
        points = []
        failed_buckets = []
        for row in rows:
            duration_seconds = row.get(f"{mode}_duration_seconds")
            code = row.get(f"{mode}_code")
            if duration_seconds is None:
                failed_buckets.append(row["bucket"])
                continue
            x = x_for_bucket(row["bucket"])
            y = y_for_duration(float(duration_seconds))
            points.append((x, y))

        if points:
            path = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
            svg.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2.2" '
                f'stroke-linejoin="round" stroke-linecap="round" points="{path}"/>'
            )

        for row in rows:
            duration_seconds = row.get(f"{mode}_duration_seconds")
            if duration_seconds is not None:
                continue
            x = x_for_bucket(row["bucket"])
            y = margin_top + 8
            svg.append(
                f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3.5" fill="#9b1c1c">'
                f'<title>{escape_xml(mode)} failed at bucket {row["bucket"]}: '
                f'{escape_xml(row.get(f"{mode}_message", ""))}</title></circle>'
            )

        if failed_buckets:
            preview = ", ".join(str(bucket) for bucket in failed_buckets[:12])
            if len(failed_buckets) > 12:
                preview += ", ..."
            svg.append(
                f'<text class="fail-label" x="{margin_left + plot_width - 4}" '
                f'y="{margin_top + 18 + 14 * modes.index(mode)}" text-anchor="end">'
                f'{escape_xml(mode)} failures: {escape_xml(preview)}</text>'
            )

    svg.append("</svg>")

    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(svg))


def process_bucket(args, start, target, modes, static_result, bucket):
    departure_timestamp = bucket_timestamp(
        args.week_start_timestamp, bucket, args.bucket_size_minutes
    )
    instant = dt.datetime.fromtimestamp(departure_timestamp, tz=dt.timezone.utc)
    row = {
        "bucket": bucket,
        "bucket_label_utc": bucket_label(departure_timestamp),
        "departure_timestamp": departure_timestamp,
        "departure_iso_utc": instant.isoformat().replace("+00:00", "Z"),
    }

    for mode in modes:
        if mode == "static":
            result = static_result
        else:
            result = route(args.host, start, target, departure_timestamp, mode, args.timeout)

        row[f"{mode}_code"] = result["code"]
        row[f"{mode}_duration_seconds"] = result["duration_seconds"]
        row[f"{mode}_duration_minutes"] = (
            None
            if result["duration_seconds"] is None
            else result["duration_seconds"] / 60.0
        )
        row[f"{mode}_distance_meters"] = result["distance_meters"]
        row[f"{mode}_message"] = result["message"]

    return row


def collect_rows(args, start, target, modes):
    static_result = None
    if "static" in modes:
        static_result = route(args.host, start, target, None, "static", args.timeout)

    bucket_start, bucket_end = args.bucket_range
    buckets = list(range(bucket_start, bucket_end + 1))
    rows = []

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(process_bucket, args, start, target, modes, static_result, bucket): bucket
            for bucket in buckets
        }
        for future in as_completed(futures):
            rows.append(future.result())

    rows.sort(key=lambda row: row["bucket"])
    return rows


def default_output_paths(args):
    start_tag = f"{args.start[0]:.5f}_{args.start[1]:.5f}".replace("-", "m").replace(".", "_")
    target_tag = f"{args.target[0]:.5f}_{args.target[1]:.5f}".replace("-", "m").replace(".", "_")
    mode_tag = "_".join(args.modes)
    stem = f"temporal_bucket_graph_{mode_tag}_{start_tag}_to_{target_tag}"
    return stem + ".csv", stem + ".svg"


def build_parser():
    parser = argparse.ArgumentParser(
        description="Evaluate one route across every temporal week bucket and graph the result."
    )
    parser.add_argument("--host", default="http://127.0.0.1:5000", help="OSRM base URL")
    parser.add_argument(
        "--start", type=parse_coord, required=True, help="Source coordinate as lon,lat"
    )
    parser.add_argument(
        "--target", type=parse_coord, required=True, help="Target coordinate as lon,lat"
    )
    parser.add_argument(
        "--modes",
        type=parse_modes,
        default=DEFAULT_MODES,
        help="Comma-separated route modes: static,depart,asymmetric,overlay_asymmetric",
    )
    parser.add_argument(
        "--week-start-timestamp",
        type=int,
        default=DEFAULT_WEEK_START_TIMESTAMP,
        help="UTC timestamp for bucket 0 (should be a Monday 00:00 UTC)",
    )
    parser.add_argument(
        "--bucket-size-minutes",
        type=int,
        default=DEFAULT_BUCKET_SIZE_MINUTES,
        help="Bucket size in minutes",
    )
    parser.add_argument(
        "--week-bucket-count",
        type=int,
        default=DEFAULT_WEEK_BUCKET_COUNT,
        help="Number of buckets in the week",
    )
    parser.add_argument(
        "--bucket-range",
        type=parse_bucket_range,
        default=(0, DEFAULT_WEEK_BUCKET_COUNT - 1),
        help="Subset of buckets as start:end, inclusive",
    )
    parser.add_argument(
        "--snap",
        action="store_true",
        help="Snap source and target through the nearest service before routing",
    )
    parser.add_argument("--timeout", type=int, default=30, help="HTTP timeout in seconds")
    parser.add_argument(
        "--workers",
        type=int,
        default=8,
        help="Number of worker threads for bucket requests",
    )
    parser.add_argument(
        "--y-unit",
        choices=["minutes", "seconds"],
        default="minutes",
        help="Y-axis unit for the graph",
    )
    parser.add_argument("--output-csv", help="CSV output path")
    parser.add_argument("--output-svg", help="SVG output path")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.bucket_size_minutes <= 0:
        parser.error("--bucket-size-minutes must be > 0")
    if args.week_bucket_count <= 0:
        parser.error("--week-bucket-count must be > 0")

    bucket_start, bucket_end = args.bucket_range
    if bucket_end >= args.week_bucket_count:
        parser.error("--bucket-range end must be smaller than --week-bucket-count")

    if args.output_csv is None or args.output_svg is None:
        default_csv, default_svg = default_output_paths(args)
        if args.output_csv is None:
            args.output_csv = default_csv
        if args.output_svg is None:
            args.output_svg = default_svg

    if args.workers <= 0:
        parser.error("--workers must be > 0")

    start = args.start
    target = args.target

    if args.snap:
        snapped_start = snap_coordinate(args.host, start, args.timeout)
        snapped_target = snap_coordinate(args.host, target, args.timeout)
        if snapped_start is None or snapped_target is None:
            print("failed to snap source or target via /nearest", file=sys.stderr)
            return 1
        start = snapped_start
        target = snapped_target

    rows = collect_rows(args, start, target, args.modes)
    write_csv(args.output_csv, rows, args.modes)

    pair_label = (
        f"from {start[0]:.7f},{start[1]:.7f} "
        f"to {target[0]:.7f},{target[1]:.7f} | modes={','.join(args.modes)}"
    )
    build_svg(
        rows,
        args.modes,
        args.output_svg,
        pair_label,
        args.y_unit,
        args.bucket_size_minutes,
    )

    print(f"Wrote CSV to {args.output_csv}")
    print(f"Wrote SVG to {args.output_svg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
