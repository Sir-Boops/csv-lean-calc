#!/usr/bin/env python3
"""Estimate lean from GPX geometry; Python 3.9+, no extra packages.

Usage: python gpx_lean.py ride.gpx --smoothing 5 --output-dir results
       python gpx_lean.py ride.gpx --peak-left 35 --peak-right 30
       python gpx_lean.py ride.gpx --fps 30

Writes lean_angle_estimated.csv and speed.csv with exactly frame,value columns.
By default, saves to <input_stem>_results in the current working directory.
Use --output-dir to choose a different folder.
Without --fps, frame is the GPS sample row number starting at 0.
Use --start-offset HH:MM:SS (or seconds) to skip into the recording.
With --fps, frame N represents N/FPS seconds after the selected start time.
Without --fps, export starts at the first GPS sample at or after that time.
Smoothing and peak calibration use the full ride before trimming.
Values are linearly interpolated AFTER smoothing and peak calibration.
Missing endpoint values use 0; gaps between sections output 0 throughout.
Frames include time zero and stop at or before the final GPX timestamp.
FPS accepts decimals (29.97) or exact fractions (30000/1001).
Interpolation adds no measurement detail; align frame 0 with your footage.
Lean values are degrees; speed values default to mph (--speed-unit changes this).
Unavailable values are exported as 0 (a placeholder, not a measurement).
Both outputs use centered moving averages:
lean uses GPS-derived velocity vectors; speed uses scalar interval speeds.
The shared --smoothing window is measured in POINTS, not
seconds. It defaults to 5; use 1 to disable. Only odd positive windows are
accepted. Edges use the available points; no averaging crosses segments,
time reversals, duplicate times, or gaps greater than --max-gap seconds.

Lean assumes balanced turning on a level road: atan(v * heading_rate / g).
Positive = left; negative = right. Estimates are NOT sensor measurements.
GPS noise, road banking, rider position and smoothing affect results,
especially peaks. No estimate is emitted below --min-speed-kmh (default 5).
GPS interval speed is an average, not an instantaneous sensor reading.

Optional --peak-left / --peak-right are positive magnitudes in degrees
(strictly between 0 and 90). Each independently rescales all valid lean
estimates on that side so its whole-file peak matches the supplied reference.
References apply AFTER smoothing; missing values are exported as 0. This is peak
normalization, not recovery of measured lean or a time-aligned calibration.
GPS outliers can determine the peak. Speed values are unaffected.
"""

import argparse
import csv
import math
from bisect import bisect_right
from fractions import Fraction
from pathlib import Path
from datetime import datetime, timezone, timedelta
import xml.etree.ElementTree as ET

EARTH_RADIUS = 6371008.8
GRAVITY = 9.80665
NS = {'g': 'http://www.topografix.com/GPX/1/1'}


def read_runs(path, max_gap):
    root = ET.parse(path).getroot()
    runs = []
    for segment in root.findall('.//g:trkseg', NS):
        run = []
        for point in segment.findall('g:trkpt', NS):
            text = point.findtext('g:time', namespaces=NS)
            if not text:
                raise ValueError('Every track point must have a timestamp.')
            time = datetime.fromisoformat(text.replace('Z', '+00:00'))
            if time.tzinfo is None:
                raise ValueError('Timestamps must include a timezone.')
            lat, lon = float(point.attrib['lat']), float(point.attrib['lon'])
            if not (math.isfinite(lat) and math.isfinite(lon)
                    and -90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError('Invalid GPS coordinates.')
            if run and not 0 < (time - run[-1][0]).total_seconds() <= max_gap:
                runs.append(run)
                run = []
            run.append((time.astimezone(timezone.utc), lat, lon))
        if run:
            runs.append(run)
    if not runs:
        raise ValueError('No GPX track points found.')
    return runs


def velocity(a, b):
    """East/north velocity using great-circle distance and initial bearing."""
    dt = (b[0] - a[0]).total_seconds()
    lat1, lat2 = math.radians(a[1]), math.radians(b[1])
    dl = math.radians((b[2] - a[2] + 180) % 360 - 180)
    h = math.sin((lat2-lat1)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dl/2)**2
    distance = 2 * EARTH_RADIUS * math.asin(math.sqrt(max(0, min(1, h))))
    bearing = math.atan2(math.sin(dl)*math.cos(lat2),
                         math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(dl))
    speed = distance / dt
    return speed * math.sin(bearing), speed * math.cos(bearing)


def estimate(run, window, min_speed):
    n = len(run)
    raw = [velocity(run[i], run[i+1]) for i in range(n-1)]
    # Interval velocities live at interval midpoints, avoiding heading wrap.
    times = [((run[i][0]-run[0][0]).total_seconds() +
              (run[i+1][0]-run[0][0]).total_seconds())/2 for i in range(n-1)]
    half = window // 2
    smooth = []
    smooth_speed = []
    for i in range(len(raw)):
        values = raw[max(0, i-half):min(len(raw), i+half+1)]
        smooth.append(tuple(sum(v[k] for v in values)/len(values) for k in (0, 1)))
        smooth_speed.append(sum(math.hypot(*v) for v in values)/len(values))
    for i, point in enumerate(run):
        speed = smooth_speed[i-1] if i else None
        lean = None
        status = 'insufficient_neighbor_points'
        if 0 < i < n-1:
            dt = times[i]-times[i-1]
            east = (smooth[i][0]+smooth[i-1][0])/2
            north = (smooth[i][1]+smooth[i-1][1])/2
            accel_e = (smooth[i][0]-smooth[i-1][0])/dt
            accel_n = (smooth[i][1]-smooth[i-1][1])/dt
            v = math.hypot(east, north)
            if v < min_speed or v == 0:
                status = 'below_min_speed'
            else:
                lateral_accel = (east*accel_n-north*accel_e)/v
                lean = math.degrees(math.atan(lateral_accel/GRAVITY))
                status = 'gps_estimate_not_sensor_measurement'
        yield point[0], speed, lean, status


def positive_odd(value):
    number = int(value)
    if number < 1 or number % 2 == 0:
        raise argparse.ArgumentTypeError('smoothing must be a positive odd integer (1, 3, 5, 7, ...).')
    return number


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be a positive finite number')
    return number


def reference_angle(value):
    angle = positive_float(value)
    if angle >= 90:
        raise argparse.ArgumentTypeError('reference angle must be less than 90 degrees')
    return angle


def frame_rate(value):
    try:
        fps = Fraction(value)
    except (ValueError, ZeroDivisionError):
        raise argparse.ArgumentTypeError('FPS must be a positive number or fraction')
    if fps <= 0:
        raise argparse.ArgumentTypeError('FPS must be positive')
    return fps


def start_offset(value):
    try:
        parts = value.split(':')
        if len(parts) == 1:
            seconds = float(value)
        elif len(parts) == 3:
            hours, minutes = int(parts[0]), int(parts[1])
            secs = float(parts[2])
            if hours < 0 or not 0 <= minutes < 60 or not 0 <= secs < 60:
                raise ValueError
            seconds = hours*3600 + minutes*60 + secs
        else:
            raise ValueError
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError
        return seconds
    except (ValueError, OverflowError):
        raise argparse.ArgumentTypeError('start offset must be nonnegative seconds or HH:MM:SS')


def video_rows(sections, fps, start=None):
    """Yield (frame, speed, lean) at exact frame times; never bridge sections."""
    previous = None
    for rows in sections:
        if previous is not None and rows[0][0] < previous:
            raise ValueError('FPS export requires chronological, non-overlapping track sections.')
        previous = rows[-1][0]
    start = sections[0][0][0] if start is None else start

    def seconds(time):
        delta = time - start
        return Fraction((delta.days*86400 + delta.seconds)*1000000 + delta.microseconds, 1000000)

    times = [[seconds(row[0]) for row in rows] for rows in sections]
    last_frame = int(times[-1][-1]*fps)
    section = 0
    for frame in range(last_frame+1):
        time = Fraction(frame, 1)/fps
        while section+1 < len(sections) and time > times[section][-1]:
            section += 1
        stamps = times[section]
        rows = sections[section]
        if time < stamps[0] or time > stamps[-1]:
            yield frame, 0, 0
            continue
        i = bisect_right(stamps, time)-1
        if stamps[i] == time:
            yield frame, rows[i][1] or 0, rows[i][2] or 0
            continue
        weight = float((time-stamps[i])/(stamps[i+1]-stamps[i]))
        values = [(rows[i][k] or 0)*(1-weight) + (rows[i+1][k] or 0)*weight for k in (1, 2)]
        yield frame, values[0], values[1]


def calibrate_peaks(sections, peak_left=None, peak_right=None):
    """Rescale each signed side independently, preserving blanks and speed."""
    angles = [row[2] for rows in sections for row in rows if row[2] is not None]
    left = max((a for a in angles if a > 0), default=0)
    right = max((-a for a in angles if a < 0), default=0)
    if peak_left is not None and left == 0:
        raise ValueError('Cannot apply --peak-left: no valid left lean estimates.')
    if peak_right is not None and right == 0:
        raise ValueError('Cannot apply --peak-right: no valid right lean estimates.')
    left_scale = peak_left/left if peak_left is not None else 1
    right_scale = peak_right/right if peak_right is not None else 1
    return [[(time, speed, None if lean is None else
              lean*(left_scale if lean > 0 else right_scale), status)
             for time, speed, lean, status in rows] for rows in sections]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('gpx', type=Path)
    parser.add_argument('--start-offset', type=start_offset, default=0, help='skip elapsed time from first GPX timestamp: HH:MM:SS or seconds (default 0)')
    parser.add_argument('--smoothing', type=positive_odd, default=5)
    parser.add_argument('--fps', type=frame_rate, help='video FPS, e.g. 30, 29.97, or 30000/1001; omit for one row per GPS sample')
    parser.add_argument('--output-dir', type=Path, help='output folder (default: <input_stem>_results in current directory)')
    parser.add_argument('--speed-unit', choices=['mph', 'kmh', 'mps'], default='mph', help='unit for speed CSV values (default mph)')
    parser.add_argument('--max-gap', type=positive_float, default=10, help='split on longer gaps in seconds (default 10)')
    parser.add_argument('--min-speed-kmh', type=positive_float, default=5, help='minimum smoothed speed for lean estimate (default 5)')
    parser.add_argument('--peak-left', type=reference_angle, help='reference peak left lean, positive degrees; rescales left estimates')
    parser.add_argument('--peak-right', type=reference_angle, help='reference peak right lean, positive degrees; rescales right estimates')
    args = parser.parse_args()
    if args.output_dir is None:
        args.output_dir = Path(f'{args.gpx.stem}_results')
    try:
        runs = read_runs(args.gpx, args.max_gap)
        sections = [list(estimate(run, args.smoothing, args.min_speed_kmh/3.6)) for run in runs]
        sections = calibrate_peaks(sections, args.peak_left, args.peak_right)
        start = sections[0][0][0] + timedelta(seconds=args.start_offset)
        if start > max(rows[-1][0] for rows in sections):
            raise ValueError('Start offset is beyond the end of the GPX recording.')
        if args.fps is not None:
            # Validate the complete timeline before opening output files.
            previous = None
            for rows in sections:
                if previous is not None and rows[0][0] < previous:
                    raise ValueError('FPS export requires chronological, non-overlapping track sections.')
                previous = rows[-1][0]
    except (OSError, ET.ParseError, ValueError, KeyError, OverflowError) as error:
        parser.error(str(error))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lean_path = args.output_dir / 'lean_angle_estimated.csv'
    speed_path = args.output_dir / 'speed.csv'
    count = 0
    with lean_path.open('w', newline='', encoding='utf-8') as lf, speed_path.open('w', newline='', encoding='utf-8') as sf:
        lw, sw = csv.writer(lf), csv.writer(sf)
        lw.writerow(['frame', 'value'])
        sw.writerow(['frame', 'value'])
        speed_factor = {'mph': 2.2369362921, 'kmh': 3.6, 'mps': 1}[args.speed_unit]
        output_rows = (video_rows(sections, args.fps, start) if args.fps is not None else
                       ((i, row[1], row[2]) for i, row in enumerate(row for rows in sections for row in rows if row[0] >= start)))
        for frame, speed, lean in output_rows:
            lw.writerow([frame, 0 if lean is None else round(lean, 3)])
            sw.writerow([frame, 0 if speed is None else round(speed*speed_factor, 3)])
            count += 1
    print(f'Wrote {count} rows per file:\n{lean_path}\n{speed_path}')
    print('Lean is a GPS-based estimate. Positive = left, negative = right.')
    print(f'Speed unit: {args.speed_unit}. ' +
          (f'Video FPS: {args.fps}; frame 0 = {start.isoformat()}.' if args.fps is not None else
           'Frame = GPS sample index, starting at 0.'))
    if args.peak_left is not None or args.peak_right is not None:
        print(f'Peak references: left={args.peak_left}, right={args.peak_right} degrees (None = unscaled).')


if __name__ == '__main__':
    main()
