# CSV Lean Calc

Convert timestamped GPX tracks into estimated motorcycle lean angle and GPS speed CSVs for video overlays.

## Requirements

Python 3.9 or newer on macOS, Linux, or Windows. No additional packages required.

## Usage

Download this repository, open a terminal in its folder, and run:

```bash
python3 gpx_lean.py "your_ride.gpx"
```

Outputs go to `your_ride_results` in the current directory:

- `lean_angle_estimated.csv`: degrees; positive is left, negative is right.
- `speed.csv`: mph by default.

Both files contain exactly `frame,value`. Frames start at 0. Unavailable values are exported as 0.

### Video frames and start offset

```bash
python3 gpx_lean.py "your_ride.gpx" --fps 30 --start-offset 02:00:00
```

This skips two hours from the first GPX timestamp and starts output at frame 0. For example, a ride beginning at 1:00 starts exporting at 3:00. The offset is elapsed time, not a clock time; seconds such as `7200` also work.

With FPS enabled, values are interpolated at video-frame intervals. Fractional rates such as `29.97` and `30000/1001` are supported. Frame 0 corresponds to the selected start time. Recording gaps output zero rather than interpolating across sections.

Without `--fps`, each row represents a GPS sample; after an offset, output begins at the first sample at or after the selected time.

### Smoothing and peak references

```bash
python3 gpx_lean.py "your_ride.gpx" --fps 30 --smoothing 5 --peak-left 35 --peak-right 30
```

Both outputs default to centered 5-point smoothing. Use an odd positive window; `1` disables smoothing. Lean smooths velocity vectors; speed smooths scalar interval speeds. Smoothing does not cross track sections or recording gaps.

Peak references independently scale the full ride's maximum left and right estimates to the supplied positive magnitudes. Smoothing and calibration happen before trimming: changing the start offset does not change the remaining samples' calibration. A referenced peak before the selected start may be absent from the exported CSV.

## Options

| Option | Default | Meaning |
| --- | --- | --- |
| `--fps` | omitted | Video frame rate; omit for one row per GPS sample |
| `--start-offset` | `0` | Elapsed seconds or `HH:MM:SS` to skip |
| `--smoothing` | `5` | Odd positive smoothing window in points |
| `--peak-left` | unscaled | Positive reference angle, below 90 degrees |
| `--peak-right` | unscaled | Positive reference angle, below 90 degrees |
| `--speed-unit` | `mph` | `mph`, `kmh`, or `mps` |
| `--output-dir` | `<input_stem>_results` | Output directory |
| `--max-gap` | `10` | Split runs on gaps longer than this many seconds |
| `--min-speed-kmh` | `5` | Minimum smoothed speed for lean estimation |

Run `python3 gpx_lean.py --help` for built-in documentation.

## How the estimate works

Lean is estimated from GPS-derived velocity and changes in direction, assuming balanced turning on a level road. It is not measured bike lean. GPS noise, road banking, rider position, sparse samples and smoothing affect the result. Peak references normalize the estimate; they do not recover actual measurements. Video interpolation adds smooth transitions without adding measurement detail.

Input must contain GPX 1.1 track points with timezone-aware timestamps. FPS export requires chronological, non-overlapping track sections; shared boundary timestamps are allowed.
