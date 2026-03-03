"""Track definitions: cone positions, centerline, and boundary geometry.

Each track is a ``Track`` dataclass that precomputes its centerline and
boundary segments in ``__post_init__``.  Built-in tracks:

* **oval** — the original fs_track.sdf oval (R≈20 m)
* **hairpin** — complex track with left sweeper + S-chicane (both L/R turns)
* **autocross** — larger (~80×60 m), gentler curves, mixed L/R turns

Tracks can also be loaded from JSON or generated randomly via
``random-track-generator`` (lazy import — only needed for generation).

Backward-compatible module-level aliases (``BLUE_CONES``, ``CENTERLINE_XY``,
``project_to_centerline``, …) point to ``OVAL_TRACK`` so that files that
have not been updated yet continue to work.
"""

from __future__ import annotations

import json as _json
import os
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

NUM_INTERP_PER_SEGMENT = 100


# ---------------------------------------------------------------------------
# Track dataclass
# ---------------------------------------------------------------------------

@dataclass
class Track:
    """A closed track defined by blue (left-of-travel) and yellow (right-of-travel) cones."""

    name: str
    blue_cones: np.ndarray
    yellow_cones: np.ndarray
    orange_cones: np.ndarray
    spawn_x: float
    spawn_y: float
    spawn_yaw: float
    half_track_width: float = 1.5

    # Computed in __post_init__
    centerline_xy: np.ndarray = field(init=False, repr=False)
    centerline_s: np.ndarray = field(init=False, repr=False)
    track_length: float = field(init=False)
    _all_starts: np.ndarray = field(init=False, repr=False)
    _all_ends: np.ndarray = field(init=False, repr=False)
    _all_dx: np.ndarray = field(init=False, repr=False)
    _all_dy: np.ndarray = field(init=False, repr=False)
    _all_len_sq: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        self._compute_centerline()
        self._compute_boundary_segments()

    # ── centerline ────────────────────────────────────────────────────

    def _compute_centerline(self):
        midpoints = (self.blue_cones + self.yellow_cones) / 2.0
        spawn_dists = np.hypot(
            midpoints[:, 0] - self.spawn_x,
            midpoints[:, 1] - self.spawn_y,
        )
        spawn_idx = int(np.argmin(spawn_dists))
        ordered = np.roll(midpoints, -spawn_idx, axis=0)

        n = len(ordered)
        points = []
        cum_s = [0.0]
        for i in range(n):
            p0 = ordered[i]
            p1 = ordered[(i + 1) % n]
            for j in range(NUM_INTERP_PER_SEGMENT):
                t = j / NUM_INTERP_PER_SEGMENT
                x = p0[0] + t * (p1[0] - p0[0])
                y = p0[1] + t * (p1[1] - p0[1])
                points.append((x, y))
                if len(points) > 1:
                    dx = points[-1][0] - points[-2][0]
                    dy = points[-1][1] - points[-2][1]
                    cum_s.append(cum_s[-1] + np.hypot(dx, dy))

        self.centerline_xy = np.array(points)
        self.centerline_s = np.array(cum_s)
        self.track_length = self.centerline_s[-1] + np.hypot(
            self.centerline_xy[0, 0] - self.centerline_xy[-1, 0],
            self.centerline_xy[0, 1] - self.centerline_xy[-1, 1],
        )

    # ── boundary segments ─────────────────────────────────────────────

    def _compute_boundary_segments(self):
        blue_starts = self.blue_cones
        blue_ends = np.roll(self.blue_cones, -1, axis=0)
        yellow_starts = self.yellow_cones
        yellow_ends = np.roll(self.yellow_cones, -1, axis=0)
        self._all_starts = np.vstack([blue_starts, yellow_starts])
        self._all_ends = np.vstack([blue_ends, yellow_ends])
        self._all_dx = self._all_ends[:, 0] - self._all_starts[:, 0]
        self._all_dy = self._all_ends[:, 1] - self._all_starts[:, 1]
        self._all_len_sq = self._all_dx ** 2 + self._all_dy ** 2

    # ── projection ────────────────────────────────────────────────────

    def project_to_centerline(self, x: float, y: float) -> Tuple[float, float]:
        """Nearest centerline point → (arc-length *s*, cross-track error)."""
        dx = self.centerline_xy[:, 0] - x
        dy = self.centerline_xy[:, 1] - y
        dists_sq = dx * dx + dy * dy
        idx = np.argmin(dists_sq)
        return float(self.centerline_s[idx]), float(np.sqrt(dists_sq[idx]))

    # ── boundary queries ──────────────────────────────────────────────

    @staticmethod
    def _point_in_polygon(px, py, poly_x, poly_y) -> bool:
        y1 = poly_y
        y2 = np.roll(poly_y, -1)
        x1 = poly_x
        x2 = np.roll(poly_x, -1)
        cond1 = (y1 > py) != (y2 > py)
        x_intersect = (x2 - x1) * (py - y1) / (y2 - y1) + x1
        cond2 = px < x_intersect
        return int(np.sum(cond1 & cond2)) % 2 == 1

    def is_inside_track(self, x: float, y: float) -> bool:
        """True if (x, y) is on the track surface (between boundaries).

        Uses a local approach: for the nearest blue and yellow boundary
        segments, checks that the point is on the track-interior side of
        each using cross products.  Robust for complex / non-convex tracks.
        """
        # --- helper: signed distance to a polyline (positive = left of travel)
        def _side_of_nearest(cones, px, py):
            starts = cones
            ends = np.roll(cones, -1, axis=0)
            dx_s = ends[:, 0] - starts[:, 0]
            dy_s = ends[:, 1] - starts[:, 1]
            len_sq = dx_s * dx_s + dy_s * dy_s
            apx = px - starts[:, 0]
            apy = py - starts[:, 1]
            t = np.clip((apx * dx_s + apy * dy_s) / len_sq, 0.0, 1.0)
            proj_x = starts[:, 0] + t * dx_s
            proj_y = starts[:, 1] + t * dy_s
            dists = np.hypot(px - proj_x, py - proj_y)
            idx = int(np.argmin(dists))
            # Cross product: positive → point is left of segment direction
            cross = dx_s[idx] * (py - starts[idx, 1]) - dy_s[idx] * (px - starts[idx, 0])
            return float(dists[idx]), float(cross)

        blue_dist, blue_cross = _side_of_nearest(self.blue_cones, x, y)
        yellow_dist, yellow_cross = _side_of_nearest(self.yellow_cones, x, y)

        # Blue cones are on the LEFT of travel direction → inside is to the RIGHT
        # (cross < 0 means right of blue boundary → inside)
        # Yellow cones are on the RIGHT → inside is to the LEFT
        # (cross > 0 means left of yellow boundary → inside)
        return blue_cross <= 0 and yellow_cross >= 0

    def dist_to_boundary(self, x: float, y: float) -> float:
        """Signed distance to nearest boundary segment (+inside, −outside)."""
        apx = x - self._all_starts[:, 0]
        apy = y - self._all_starts[:, 1]
        t = (apx * self._all_dx + apy * self._all_dy) / self._all_len_sq
        t = np.clip(t, 0.0, 1.0)
        proj_x = self._all_starts[:, 0] + t * self._all_dx
        proj_y = self._all_starts[:, 1] + t * self._all_dy
        dists = np.hypot(x - proj_x, y - proj_y)
        d = float(dists.min())
        if not self.is_inside_track(x, y):
            return -d
        return d


# ---------------------------------------------------------------------------
# Oval track — extracted from fs_track.sdf
# ---------------------------------------------------------------------------

_OVAL_BLUE = np.array([
    # Right straight (inner, x=18.5)
    [18.5, -10], [18.5, -5], [18.5, 0], [18.5, 5], [18.5, 10],
    # Top curve (inner, r=18.5 from center (0,10))
    [16.02, 14.25], [9.25, 18.02], [0, 20], [-9.25, 18.02], [-16.02, 14.25],
    # Left straight (inner, x=-18.5)
    [-18.5, 10], [-18.5, 5], [-18.5, 0], [-18.5, -5], [-18.5, -10],
    # Bottom curve (inner, r=18.5 from center (0,-10))
    [-16.02, -14.25], [-9.25, -18.02], [0, -20], [9.25, -18.02], [16.02, -14.25],
])

_OVAL_YELLOW = np.array([
    # Right straight (outer, x=21.5)
    [21.5, -10], [21.5, -5], [21.5, 0], [21.5, 5], [21.5, 10],
    # Top curve (outer, r=21.5 from center (0,10))
    [18.62, 16.75], [10.75, 20.62], [0, 23], [-10.75, 20.62], [-18.62, 16.75],
    # Left straight (outer, x=-21.5)
    [-21.5, 10], [-21.5, 5], [-21.5, 0], [-21.5, -5], [-21.5, -10],
    # Bottom curve (outer, r=21.5 from center (0,-10))
    [-18.62, -16.75], [-10.75, -20.62], [0, -23], [10.75, -20.62], [18.62, -16.75],
])

_OVAL_ORANGE = np.array([
    [18.0, -0.5], [18.0, 0.5], [22.0, -0.5], [22.0, 0.5],
])

OVAL_TRACK = Track(
    name="oval",
    blue_cones=_OVAL_BLUE,
    yellow_cones=_OVAL_YELLOW,
    orange_cones=_OVAL_ORANGE,
    spawn_x=20.0, spawn_y=0.0, spawn_yaw=np.pi / 2,
    half_track_width=1.5,
)


# ---------------------------------------------------------------------------
# Complex track — left sweeper + S-chicane with both LEFT and RIGHT turns
# ---------------------------------------------------------------------------
# Generated from centerline waypoints via cubic spline + perpendicular offsets.
# Blue = left-of-travel, Yellow = right-of-travel.
# The S-chicane section forces the kart to make RIGHT turns, preventing
# overfitting to counterclockwise-only loops.
# Track width = 3 m.  ~90 m centerline length.  Fits in ~40×35 m area.


def _generate_cones_from_centerline(waypoints, half_width=1.5, n_cones=25):
    """Generate blue (left) and yellow (right) cones from centerline waypoints.

    Args:
        waypoints: (N, 2) closed loop (first != last, auto-closed).
        half_width: offset distance from centerline to each boundary.
        n_cones: number of cones per side.

    Returns:
        (blue_cones, yellow_cones) each (n_cones, 2).
    """
    from scipy.interpolate import CubicSpline

    pts = np.vstack([waypoints, waypoints[0:1]])
    diffs = np.diff(pts, axis=0)
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    t = np.zeros(len(pts))
    t[1:] = np.cumsum(seg_len)

    cs_x = CubicSpline(t, pts[:, 0], bc_type='periodic')
    cs_y = CubicSpline(t, pts[:, 1], bc_type='periodic')

    n_fine = 2000
    t_fine = np.linspace(0, t[-1], n_fine, endpoint=False)
    cx = cs_x(t_fine)
    cy = cs_y(t_fine)

    dx = cs_x(t_fine, 1)
    dy = cs_y(t_fine, 1)
    mag = np.hypot(dx, dy)
    nx = -dy / mag   # left-of-travel normal
    ny = dx / mag

    blue_x = cx + half_width * nx
    blue_y = cy + half_width * ny
    yellow_x = cx - half_width * nx
    yellow_y = cy - half_width * ny

    idx = np.linspace(0, n_fine, n_cones, endpoint=False).astype(int)
    blue = np.column_stack([blue_x[idx], blue_y[idx]])
    yellow = np.column_stack([yellow_x[idx], yellow_y[idx]])
    return blue, yellow


# Centerline waypoints: oval + gentle chicane on right straight (forces right turns)
# The chicane offsets the track 5m to the right then back, creating clear R/L turns.
_COMPLEX_WAYPOINTS = np.array([
    # Right straight heading north (x=20)
    [20, -8],
    [20, -3],
    # Chicane: gentle 5m jog right
    [25, 1],
    [25, 5],
    # Chicane: jog back left
    [20, 9],
    [20, 13],
    # Top curve (left turn)
    [14, 20],
    [6, 23],
    [-6, 23],
    [-14, 20],
    # Left straight heading south (x=-20)
    [-20, 13],
    [-20, 5],
    [-20, -3],
    [-20, -8],
    # Bottom curve (left turn)
    [-14, -15],
    [-6, -18],
    [6, -18],
    [14, -15],
])

_COMPLEX_BLUE, _COMPLEX_YELLOW = _generate_cones_from_centerline(
    _COMPLEX_WAYPOINTS, half_width=1.5, n_cones=50,
)

_COMPLEX_ORANGE = np.array([
    [19.0, -6.0], [19.0, -5.0], [21.0, -6.0], [21.0, -5.0],
])

HAIRPIN_TRACK = Track(
    name="hairpin",
    blue_cones=_COMPLEX_BLUE,
    yellow_cones=_COMPLEX_YELLOW,
    orange_cones=_COMPLEX_ORANGE,
    spawn_x=20.0, spawn_y=-5.5, spawn_yaw=np.pi / 2,
    half_track_width=1.5,
)


# ---------------------------------------------------------------------------
# Autocross track — larger (~80×60 m), gentler curves, mixed L/R turns
# ---------------------------------------------------------------------------
# Inspired by Formula Student autocross layouts.
# ~250 m lap, minimum turn radius ~10 m, track width 3 m.

_AUTOCROSS_WAYPOINTS = np.array([
    # Start/finish straight heading north (east side)
    [30, -12],
    [30,  -2],
    [30,   8],
    # Right turn into northern section (R~15m)
    [25,  20],
    [15,  25],
    # Gentle S-chicane (direction changes)
    [ 5,  22],
    [-5,  26],
    [-15, 22],
    # Left sweeper heading west
    [-25, 15],
    [-35, 10],
    # Wide left curve at west end (R~15m)
    [-40,  0],
    [-35, -10],
    [-25, -18],
    # Bottom straight heading east
    [-10, -22],
    [  5, -22],
    [ 15, -22],
    # Wide right curve back to start
    [ 25, -18],
])

_AUTOCROSS_BLUE, _AUTOCROSS_YELLOW = _generate_cones_from_centerline(
    _AUTOCROSS_WAYPOINTS, half_width=1.5, n_cones=70,
)

_AUTOCROSS_ORANGE = np.array([
    [29.0, -8.0], [29.0, -7.0], [31.0, -8.0], [31.0, -7.0],
])

AUTOCROSS_TRACK = Track(
    name="autocross",
    blue_cones=_AUTOCROSS_BLUE,
    yellow_cones=_AUTOCROSS_YELLOW,
    orange_cones=_AUTOCROSS_ORANGE,
    spawn_x=30.0, spawn_y=-7.5, spawn_yaw=np.pi / 2,
    half_track_width=1.5,
)


# ---------------------------------------------------------------------------
# Random track generation (lazy import of random-track-generator)
# ---------------------------------------------------------------------------

def _resample_boundary(cones: np.ndarray, n: int) -> np.ndarray:
    """Interpolate a closed cone boundary to *n* evenly-spaced points."""
    closed = np.vstack([cones, cones[0:1]])
    diffs = np.diff(closed, axis=0)
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    cum = np.zeros(len(closed))
    cum[1:] = np.cumsum(seg_len)
    total = cum[-1]
    targets = np.linspace(0, total, n, endpoint=False)
    out = np.empty((n, 2))
    for i, s in enumerate(targets):
        idx = int(np.searchsorted(cum, s, side="right")) - 1
        idx = min(idx, len(cones) - 1)
        frac = (s - cum[idx]) / seg_len[idx] if seg_len[idx] > 0 else 0.0
        out[i] = closed[idx] + frac * (closed[idx + 1] - closed[idx])
    return out


def _generate_in_subprocess(n_points, n_regions, max_bound, mode, seed, timeout=30):
    """Run generate_track in a subprocess to handle seeds that hang."""
    import subprocess
    import sys

    code = (
        f"import json, numpy as np\n"
        f"from random_track_generator import generate_track\n"
        f"t=generate_track(n_points={n_points},n_regions={n_regions},"
        f"min_bound=5,max_bound={max_bound},mode={mode!r},seed={seed!r})\n"
        f"json.dump(dict(left=np.asarray(t.cones_left).tolist(),"
        f"right=np.asarray(t.cones_right).tolist()),__import__('sys').stdout)"
    )
    try:
        r = subprocess.run(
            [sys.executable, "-c", code],
            capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError(
            f"random-track-generator hung for seed={seed} (timeout={timeout}s). "
            f"Try a different seed."
        )
    if r.returncode != 0:
        raise RuntimeError(f"random-track-generator failed: {r.stderr.strip()[:200]}")
    data = _json.loads(r.stdout)
    return np.array(data["left"]), np.array(data["right"])


def random_track(
    seed: int | None = None,
    mode: str = "expand",
    n_points: int = 5,
    n_regions: int = 2,
    max_bound: float = 40.0,
) -> Track:
    """Generate a random track using ``random-track-generator``.

    Requires ``pip install random-track-generator``.

    Note: some seed/param combinations cause the generator to hang.
    A 30 s subprocess timeout is used; try a different seed if it fails.
    """
    left, right = _generate_in_subprocess(n_points, n_regions, max_bound, mode, seed)

    # Resample both to equal length (use max of the two)
    n_cones = max(len(left), len(right))
    blue = _resample_boundary(left, n_cones)
    yellow = _resample_boundary(right, n_cones)

    # Spawn: midpoint of first cone pair, heading toward second pair
    spawn = (blue[0] + yellow[0]) / 2.0
    next_mid = (blue[1] + yellow[1]) / 2.0
    d = next_mid - spawn
    spawn_yaw = float(np.arctan2(d[1], d[0]))

    # Orange cones at start/finish (2 per side of start line)
    perp = np.array([-np.sin(spawn_yaw), np.cos(spawn_yaw)])
    hw = np.median(np.linalg.norm(blue - yellow, axis=1)) / 2.0
    orange = np.array([
        spawn + hw * perp - 0.5 * np.array([np.cos(spawn_yaw), np.sin(spawn_yaw)]),
        spawn + hw * perp + 0.5 * np.array([np.cos(spawn_yaw), np.sin(spawn_yaw)]),
        spawn - hw * perp - 0.5 * np.array([np.cos(spawn_yaw), np.sin(spawn_yaw)]),
        spawn - hw * perp + 0.5 * np.array([np.cos(spawn_yaw), np.sin(spawn_yaw)]),
    ])

    name = f"random_s{seed}" if seed is not None else "random"
    return Track(
        name=name,
        blue_cones=blue,
        yellow_cones=yellow,
        orange_cones=orange,
        spawn_x=float(spawn[0]),
        spawn_y=float(spawn[1]),
        spawn_yaw=spawn_yaw,
        half_track_width=float(hw),
    )


# ---------------------------------------------------------------------------
# JSON serialization
# ---------------------------------------------------------------------------

def track_to_json(track: Track, path: str) -> None:
    """Save a track to a JSON file for reproducibility."""
    data = {
        "name": track.name,
        "blue_cones": track.blue_cones.tolist(),
        "yellow_cones": track.yellow_cones.tolist(),
        "orange_cones": track.orange_cones.tolist(),
        "spawn_x": track.spawn_x,
        "spawn_y": track.spawn_y,
        "spawn_yaw": track.spawn_yaw,
        "half_track_width": track.half_track_width,
    }
    with open(path, "w") as f:
        _json.dump(data, f, indent=2)


def track_from_json(path: str) -> Track:
    """Load a track from a JSON file."""
    with open(path) as f:
        data = _json.load(f)
    return Track(
        name=data.get("name", os.path.basename(path)),
        blue_cones=np.array(data["blue_cones"]),
        yellow_cones=np.array(data["yellow_cones"]),
        orange_cones=np.array(data["orange_cones"]),
        spawn_x=data["spawn_x"],
        spawn_y=data["spawn_y"],
        spawn_yaw=data["spawn_yaw"],
        half_track_width=data.get("half_track_width", 1.5),
    )


# ---------------------------------------------------------------------------
# Registry / lookup
# ---------------------------------------------------------------------------

TRACKS = {
    "oval": OVAL_TRACK,
    "hairpin": HAIRPIN_TRACK,
    "autocross": AUTOCROSS_TRACK,
}


def _parse_random_spec(spec: str) -> dict:
    """Parse ``"random:seed=42,mode=expand,max_bound=80"`` → kwargs dict."""
    kwargs: dict = {}
    if ":" not in spec:
        return kwargs
    params = spec.split(":", 1)[1]
    for part in params.split(","):
        k, _, v = part.partition("=")
        k = k.strip()
        v = v.strip()
        if k == "seed":
            kwargs["seed"] = int(v)
        elif k == "mode":
            kwargs["mode"] = v
        elif k == "n_points":
            kwargs["n_points"] = int(v)
        elif k == "n_regions":
            kwargs["n_regions"] = int(v)
        elif k == "max_bound":
            kwargs["max_bound"] = float(v)
    return kwargs


def get_track(name: str) -> Track:
    """Look up a track by name, JSON path, or random spec.

    Accepted forms:
    - Built-in name: ``"oval"``, ``"hairpin"``, ``"autocross"``
    - JSON file path: ``"path/to/track.json"``
    - Random: ``"random"`` or ``"random:seed=42,mode=expand,max_bound=80"``
    """
    if name in TRACKS:
        return TRACKS[name]
    if name.endswith(".json"):
        return track_from_json(name)
    if name == "random" or name.startswith("random:"):
        kwargs = _parse_random_spec(name)
        return random_track(**kwargs)
    raise KeyError(
        f"Unknown track: {name!r}. "
        f"Use one of {list(TRACKS)}, a .json path, or 'random[:opts]'"
    )


# ---------------------------------------------------------------------------
# Backward-compatible module-level aliases → OVAL_TRACK
# ---------------------------------------------------------------------------

BLUE_CONES = OVAL_TRACK.blue_cones
YELLOW_CONES = OVAL_TRACK.yellow_cones
ORANGE_CONES = OVAL_TRACK.orange_cones
SPAWN_X = OVAL_TRACK.spawn_x
SPAWN_Y = OVAL_TRACK.spawn_y
SPAWN_YAW = OVAL_TRACK.spawn_yaw
CENTERLINE_XY = OVAL_TRACK.centerline_xy
CENTERLINE_S = OVAL_TRACK.centerline_s
TRACK_LENGTH = OVAL_TRACK.track_length
HALF_TRACK_WIDTH = OVAL_TRACK.half_track_width

project_to_centerline = OVAL_TRACK.project_to_centerline
is_inside_track = OVAL_TRACK.is_inside_track
dist_to_boundary = OVAL_TRACK.dist_to_boundary
