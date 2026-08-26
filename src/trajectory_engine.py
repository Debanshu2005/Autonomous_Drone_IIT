"""
Natural-language command parsing and NED/global waypoint generation.

All local geometry is expressed in NED convention:
- x / north is positive forward toward geographic north.
- y / east is positive toward east.
- altitude above launch is represented as ``down_m = -altitude_m``.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from sensor_check import NavigationMode, SensorReport


EARTH_RADIUS_M = 6378137.0
DISTANCE_UNIT_PATTERN = r"(?:m|meter|meters|metre|metres)?"
DISTANCE_UNIT_WORD_PATTERN = r"(?:meters|meter|metres|metre|m)"
TIME_UNIT_PATTERN = r"(?:seconds|second|secs|sec|s)"


class TaskAction(str, Enum):
    CIRCLE = "circle"
    GOTO = "goto"
    SQUARE = "square"
    HOVER = "hover"
    SET_MODE = "set_mode"
    HOLD = "hold"
    LAND = "land"
    RTL = "rtl"
    TAKEOFF = "takeoff"
    TAKEOFF_LAND = "takeoff_land"


class TargetFrame(str, Enum):
    GLOBAL_RELATIVE_ALT = "global_relative_alt"
    LOCAL_NED = "local_ned"


@dataclass(frozen=True)
class ParsedTask:
    action: TaskAction
    params: dict[str, float]
    raw_text: str
    notes: tuple[str, ...] = ()


@dataclass(frozen=True)
class TaskSequence:
    tasks: list[ParsedTask]
    raw_text: str
    notes: tuple[str, ...] = ()

    @property
    def action_names(self) -> list[str]:
        return [task.action.name for task in self.tasks]


@dataclass(frozen=True)
class LocalTarget:
    name: str
    north_m: float
    east_m: float
    down_m: float
    hold_s: float = 0.0
    yaw_deg: Optional[float] = None

    @property
    def altitude_m(self) -> float:
        return -self.down_m


@dataclass(frozen=True)
class GlobalTarget:
    name: str
    lat_deg: float
    lon_deg: float
    relative_alt_m: float
    hold_s: float = 0.0
    yaw_deg: Optional[float] = None


@dataclass(frozen=True)
class VehicleOrigin:
    local_north_m: float
    local_east_m: float
    local_down_m: float
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    relative_alt_m: Optional[float] = None


@dataclass(frozen=True)
class TrajectoryPlan:
    action: TaskAction
    frame: TargetFrame
    local_targets: list[LocalTarget]
    global_targets: list[GlobalTarget]
    target_altitude_m: float
    description: str

    @property
    def count(self) -> int:
        return len(self.local_targets) if self.frame == TargetFrame.LOCAL_NED else len(self.global_targets)


def parse_task(text: str, default_altitude_m: float = 3.0) -> ParsedTask:
    normalized = _normalize_text(text)
    if not normalized:
        raise ValueError("empty command")
    normalized = _strip_command_prefix(normalized)

    params = _parse_key_values(normalized)
    numbers = _numbers(normalized)

    mode_name = _mode_request(normalized)
    if mode_name:
        return ParsedTask(TaskAction.SET_MODE, {}, text, (f"mode={mode_name}",))

    if "hover" in normalized and "takeoff" not in normalized:
        hover_s = _duration_seconds(normalized)
        if hover_s is None:
            hover_s = _first_or(5.0, numbers)
        return ParsedTask(TaskAction.HOVER, {"hover_s": hover_s}, text)
    if normalized in {"hold", "hold position", "loiter", "pause"}:
        return ParsedTask(TaskAction.HOLD, params, text)
    if normalized in {"land", "land now", "abort land"}:
        return ParsedTask(TaskAction.LAND, params, text)
    if normalized in {"rtl", "return", "return home", "return to launch"}:
        return ParsedTask(TaskAction.RTL, params, text)

    if "takeoff" in normalized or "take off" in normalized or "lift off" in normalized:
        params.setdefault("h", _takeoff_altitude(normalized, default_altitude_m))
        params.setdefault("hover_s", _duration_seconds(normalized) or 0.0)
        action = TaskAction.TAKEOFF_LAND if "land" in normalized else TaskAction.TAKEOFF
        return ParsedTask(action, params, text)

    if "circle" in normalized or "orbit" in normalized:
        params.setdefault("r", _named_distance(normalized, ("radius", "r")) or _first_or(5.0, numbers))
        params.setdefault("h", _named_distance(normalized, ("altitude", "height", "alt", "h")) or default_altitude_m)
        params.setdefault("n", 36.0)
        return ParsedTask(TaskAction.CIRCLE, params, text)

    if "square" in normalized or "box" in normalized or "search pattern" in normalized:
        params.setdefault("size", _named_distance(normalized, ("size", "side", "pattern")) or _first_or(10.0, numbers))
        params.setdefault("h", _named_distance(normalized, ("altitude", "height", "alt", "h")) or default_altitude_m)
        params.setdefault("passes", 4.0)
        return ParsedTask(TaskAction.SQUARE, params, text)

    if (
        "goto" in normalized
        or "go to" in normalized
        or "fly to" in normalized
        or (
            re.search(r"\b(?:go|fly|move)\b", normalized) is not None
            and re.search(r"\b(?:north|east|south|west)\b", normalized) is not None
        )
    ):
        if "x" not in params and "n" not in params and "north" not in params:
            north = _direction_distance(normalized, "north")
            south = _direction_distance(normalized, "south")
            if north is not None:
                params["x"] = north
            elif south is not None:
                params["x"] = -south
            elif numbers:
                params["x"] = numbers[0]
        if "y" not in params and "e" not in params and "east" not in params:
            east = _direction_distance(normalized, "east")
            west = _direction_distance(normalized, "west")
            if east is not None:
                params["y"] = east
            elif west is not None:
                params["y"] = -west
            elif len(numbers) > 1:
                params["y"] = numbers[1]
        params.setdefault("h", _named_distance(normalized, ("altitude", "height", "alt", "h")) or default_altitude_m)
        return ParsedTask(TaskAction.GOTO, params, text)

    raise ValueError(
        "unknown task"
    )


def parse_task_sequence(
    text: str,
    default_altitude_m: float = 3.0,
    default_hover_s: float = 2.0,
) -> TaskSequence:
    normalized = _strip_command_prefix(_normalize_text(text))
    if not normalized:
        raise ValueError("empty command")

    if _is_directional_goto_text(normalized):
        task = parse_task(normalized, default_altitude_m)
        return TaskSequence([task], text, task.notes)

    if not _looks_compound(normalized):
        task = parse_task(normalized, default_altitude_m)
        if task.action == TaskAction.TAKEOFF_LAND:
            return TaskSequence(
                [
                    ParsedTask(TaskAction.TAKEOFF, {"h": task.params["h"]}, text, task.notes),
                    ParsedTask(TaskAction.HOVER, {"hover_s": task.params.get("hover_s", default_hover_s)}, text),
                    ParsedTask(TaskAction.LAND, {}, text),
                ],
                text,
                task.notes,
            )
        return TaskSequence([task], text, task.notes)

    tasks: list[ParsedTask] = []
    notes: list[str] = []
    last_altitude_m = default_altitude_m
    clauses = _split_clauses(normalized)
    for index, clause in enumerate(clauses):
        try:
            task = parse_task(clause, last_altitude_m)
        except ValueError:
            if "launch" in clause or "home" in clause:
                task = ParsedTask(TaskAction.RTL, {}, clause)
            else:
                raise ValueError(f"could not parse clause {index + 1}: {clause!r}")

        if task.action == TaskAction.TAKEOFF_LAND:
            tasks.append(ParsedTask(TaskAction.TAKEOFF, {"h": task.params["h"]}, clause, task.notes))
            tasks.append(ParsedTask(TaskAction.HOVER, {"hover_s": task.params.get("hover_s", default_hover_s)}, clause))
            tasks.append(ParsedTask(TaskAction.LAND, {}, clause))
            last_altitude_m = task.params["h"]
            continue

        if task.action == TaskAction.TAKEOFF:
            explicit_altitude = _has_explicit_altitude(clause)
            if not explicit_altitude:
                next_hover_altitude = _next_hover_altitude_hint(clauses[index + 1 :])
                if next_hover_altitude is not None:
                    task = ParsedTask(
                        TaskAction.TAKEOFF,
                        {**task.params, "h": next_hover_altitude},
                        clause,
                        ("Interpreted hover distance as takeoff altitude.",),
                    )
                    notes.append("Interpreted hover distance as takeoff altitude.")
            last_altitude_m = task.params.get("h", last_altitude_m)

        if task.action in {TaskAction.CIRCLE, TaskAction.GOTO, TaskAction.SQUARE}:
            task = ParsedTask(task.action, {**task.params, "h": task.params.get("h", last_altitude_m)}, clause, task.notes)

        if task.action == TaskAction.HOVER:
            hover_s = _duration_seconds(clause)
            if hover_s is None:
                hover_s = default_hover_s
                if _hover_altitude_hint(clause) is not None:
                    notes.append("Hover distance was treated as altitude; hover time defaulted to 2s.")
            task = ParsedTask(TaskAction.HOVER, {"hover_s": hover_s}, clause, task.notes)

        tasks.append(task)

    if not tasks:
        raise ValueError("no executable tasks found")
    return TaskSequence(tasks, text, tuple(dict.fromkeys(notes)))


def build_trajectory(
    task: ParsedTask,
    report: SensorReport,
    origin: VehicleOrigin,
) -> TrajectoryPlan:
    if task.action in {TaskAction.HOVER, TaskAction.SET_MODE, TaskAction.HOLD, TaskAction.LAND, TaskAction.RTL}:
        frame = _frame_for_mode(report.mode)
        altitude_m = max(0.0, float(origin.relative_alt_m or 0.0))
        if task.action == TaskAction.HOVER:
            altitude_m = max(altitude_m, float(task.params.get("h", altitude_m)))
            description = f"hover duration={task.params.get('hover_s', 0.0):.1f}s"
        elif task.action == TaskAction.SET_MODE:
            mode_name = task.notes[0].removeprefix("mode=") if task.notes else "UNKNOWN"
            description = f"set mode {mode_name}"
        else:
            description = task.action.value
        return TrajectoryPlan(task.action, frame, [], [], altitude_m, description)

    if report.mode == NavigationMode.MODE_C_DEGRADED and task.action not in {
        TaskAction.HOLD,
        TaskAction.LAND,
        TaskAction.RTL,
    }:
        raise ValueError("cannot generate navigation trajectory with degraded position estimate")

    altitude_m = _positive(task.params.get("h", origin.relative_alt_m or 3.0), "altitude")
    if task.action == TaskAction.CIRCLE:
        local_targets = _circle_targets(task, origin, altitude_m)
        description = f"circle radius={task.params['r']:.1f}m altitude={altitude_m:.1f}m"
    elif task.action == TaskAction.GOTO:
        local_targets = [_goto_target(task, origin, altitude_m)]
        description = (
            f"goto north={local_targets[0].north_m:.1f}m "
            f"east={local_targets[0].east_m:.1f}m altitude={altitude_m:.1f}m"
        )
    elif task.action == TaskAction.SQUARE:
        local_targets = _square_search_targets(task, origin, altitude_m)
        description = f"square search size={task.params['size']:.1f}m altitude={altitude_m:.1f}m"
    elif task.action in {TaskAction.TAKEOFF, TaskAction.TAKEOFF_LAND}:
        hover_s = task.params.get("hover_s", 0.0)
        suffix = " then land" if task.action == TaskAction.TAKEOFF_LAND else ""
        description = f"takeoff altitude={altitude_m:.1f}m hover={hover_s:.1f}s{suffix}"
        frame = _frame_for_mode(report.mode)
        return TrajectoryPlan(task.action, frame, [], [], altitude_m, description)
    else:
        frame = _frame_for_mode(report.mode)
        return TrajectoryPlan(task.action, frame, [], [], altitude_m, task.action.value)

    frame = _frame_for_mode(report.mode)
    global_targets: list[GlobalTarget] = []
    if frame == TargetFrame.GLOBAL_RELATIVE_ALT:
        if origin.lat_deg is None or origin.lon_deg is None:
            raise ValueError("GPS mode selected but current latitude/longitude are unavailable")
        global_targets = [
            _local_target_to_global(target, origin)
            for target in local_targets
        ]

    return TrajectoryPlan(
        action=task.action,
        frame=frame,
        local_targets=local_targets,
        global_targets=global_targets,
        target_altitude_m=altitude_m,
        description=description,
    )


def _circle_targets(task: ParsedTask, origin: VehicleOrigin, altitude_m: float) -> list[LocalTarget]:
    radius_m = _positive(task.params["r"], "radius")
    segments = max(8, min(120, int(task.params.get("n", 36))))
    center_n = origin.local_north_m
    center_e = origin.local_east_m
    targets: list[LocalTarget] = []
    for index in range(segments + 1):
        theta = 2.0 * math.pi * index / segments
        north = center_n + radius_m * math.cos(theta)
        east = center_e + radius_m * math.sin(theta)
        yaw_deg = math.degrees(theta + math.pi / 2.0)
        targets.append(LocalTarget(f"circle-{index:02d}", north, east, -altitude_m, yaw_deg=yaw_deg))
    return targets


def _goto_target(task: ParsedTask, origin: VehicleOrigin, altitude_m: float) -> LocalTarget:
    north_offset = _first_param(task.params, ("x", "n", "north"), 0.0)
    east_offset = _first_param(task.params, ("y", "e", "east"), 0.0)
    hold_s = _first_param(task.params, ("hold", "hold_s"), 0.0)
    return LocalTarget(
        "goto",
        origin.local_north_m + north_offset,
        origin.local_east_m + east_offset,
        -altitude_m,
        hold_s=hold_s,
    )


def _square_search_targets(
    task: ParsedTask,
    origin: VehicleOrigin,
    altitude_m: float,
) -> list[LocalTarget]:
    size_m = _positive(task.params["size"], "size")
    half = size_m / 2.0
    center_n = origin.local_north_m
    center_e = origin.local_east_m
    corners = [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
        (-half, -half),
    ]
    targets = [
        LocalTarget(
            f"square-{index}",
            center_n + north,
            center_e + east,
            -altitude_m,
        )
        for index, (north, east) in enumerate(corners)
    ]

    passes = max(0, int(task.params.get("passes", 0)))
    if "search" not in task.raw_text.lower() or passes <= 0:
        return targets

    lane_spacing = size_m / max(1, passes)
    lanes: list[LocalTarget] = []
    for index in range(passes + 1):
        east = center_e - half + index * lane_spacing
        if index % 2 == 0:
            north_values = (center_n - half, center_n + half)
        else:
            north_values = (center_n + half, center_n - half)
        for lane_end, north in enumerate(north_values):
            lanes.append(LocalTarget(f"search-{index}-{lane_end}", north, east, -altitude_m))
    return lanes


def _local_target_to_global(target: LocalTarget, origin: VehicleOrigin) -> GlobalTarget:
    origin_lat = float(origin.lat_deg)
    origin_lon = float(origin.lon_deg)
    delta_n = target.north_m - origin.local_north_m
    delta_e = target.east_m - origin.local_east_m
    lat, lon = offset_lat_lon(origin_lat, origin_lon, delta_n, delta_e)
    return GlobalTarget(
        target.name,
        lat,
        lon,
        target.altitude_m,
        hold_s=target.hold_s,
        yaw_deg=target.yaw_deg,
    )


def offset_lat_lon(
    origin_lat_deg: float,
    origin_lon_deg: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    lat_rad = math.radians(origin_lat_deg)
    cos_lat = max(1e-6, abs(math.cos(lat_rad)))
    d_lat = north_m / EARTH_RADIUS_M
    d_lon = east_m / (EARTH_RADIUS_M * cos_lat)
    return (
        origin_lat_deg + math.degrees(d_lat),
        origin_lon_deg + math.degrees(d_lon),
    )


def local_distance_m(a: LocalTarget, north_m: float, east_m: float, down_m: float) -> float:
    return math.sqrt((a.north_m - north_m) ** 2 + (a.east_m - east_m) ** 2 + (a.down_m - down_m) ** 2)


def global_distance_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1 = math.radians(lat1_deg)
    lat2 = math.radians(lat2_deg)
    d_lat = lat2 - lat1
    d_lon = math.radians(lon2_deg - lon1_deg)
    hav = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(hav), math.sqrt(1.0 - hav))


def _frame_for_mode(mode: NavigationMode) -> TargetFrame:
    if mode == NavigationMode.MODE_A_GPS:
        return TargetFrame.GLOBAL_RELATIVE_ALT
    return TargetFrame.LOCAL_NED


def _parse_key_values(text: str) -> dict[str, float]:
    params: dict[str, float] = {}
    for match in re.finditer(r"\b([a-z_]+)\s*=\s*([-+]?\d+(?:\.\d+)?)", text):
        params[match.group(1)] = float(match.group(2))
    return params


def _strip_command_prefix(text: str) -> str:
    return re.sub(r"^\s*\[cmd\]\s*", "", text, flags=re.IGNORECASE).strip()


def _mode_request(text: str) -> Optional[str]:
    match = re.search(
        r"\b(?:switch|swich|change|set)\s+(?:flight\s+)?mode\s+(?:to\s+)?([a-z0-9_ -]+)\b",
        text,
    )
    if match is None:
        match = re.search(r"\bmode\s+([a-z0-9_ -]+)\b", text)
    if match is None:
        return None
    raw_mode = match.group(1).strip(" .")
    return _normalize_mode_name(raw_mode)


def _normalize_mode_name(raw_mode: str) -> str:
    compact = re.sub(r"[^a-z0-9]+", "", raw_mode.lower())
    aliases = {
        "guided": "GUIDED",
        "guidednogps": "GUIDED_NOGPS",
        "althold": "ALT_HOLD",
        "altitudehold": "ALT_HOLD",
        "loiter": "LOITER",
        "stabilize": "STABILIZE",
        "land": "LAND",
        "rtl": "RTL",
        "returntolaunch": "RTL",
        "poshold": "POSHOLD",
        "positionhold": "POSHOLD",
        "offboard": "OFFBOARD",
        "hold": "HOLD",
        "altctl": "ALTCTL",
        "posctl": "POSCTL",
        "manual": "MANUAL",
    }
    return aliases.get(compact, raw_mode.strip().upper().replace("-", "_").replace(" ", "_"))


def _looks_compound(text: str) -> bool:
    action_hits = sum(
        1
        for pattern in (
            r"\btakeoff\b",
            r"\bhover\b",
            r"\bcircle\b",
            r"\borbit\b",
            r"\bsquare\b",
            r"\bsearch\b",
            r"\bgoto\b",
            r"\bgo\b",
            r"\bfly\b",
            r"\bland\b",
            r"\breturn\b",
            r"\brtl\b",
        )
        if re.search(pattern, text)
    )
    return action_hits > 1 or "," in text or " then " in text or " and " in text


def _is_directional_goto_text(text: str) -> bool:
    has_move_word = re.search(r"\b(?:goto|go|fly|move)\b", text) is not None
    has_direction = re.search(r"\b(?:north|east|south|west)\b", text) is not None
    has_sequence_word = re.search(r"\b(?:takeoff|hover|circle|orbit|square|search|land|return|rtl)\b", text) is not None
    return has_move_word and has_direction and not has_sequence_word


def _split_clauses(text: str) -> list[str]:
    cleaned = re.sub(r"\b(?:and then|then|and)\b", ",", text)
    return [clause.strip(" .") for clause in cleaned.split(",") if clause.strip(" .")]


def _has_explicit_altitude(text: str) -> bool:
    return (
        _named_distance(text, ("altitude", "height", "alt", "h")) is not None
        or re.search(
            rf"\b(?:takeoff|climb)\s*(?:to)?\s*[-+]?\d+(?:\.\d+)?\s*{DISTANCE_UNIT_PATTERN}\b",
            text,
        )
        is not None
    )


def _next_hover_altitude_hint(clauses: list[str]) -> Optional[float]:
    for clause in clauses:
        if "hover" in clause:
            return _hover_altitude_hint(clause)
        if any(word in clause for word in ("land", "rtl", "return", "circle", "goto", "go ", "fly ")):
            return None
    return None


def _hover_altitude_hint(text: str) -> Optional[float]:
    match = re.search(
        rf"\bhover\s*(?:for|at)?\s*([-+]?\d+(?:\.\d+)?)\s*{DISTANCE_UNIT_WORD_PATTERN}\b",
        text,
    )
    if match:
        return float(match.group(1))
    return None


def command_guide() -> str:
    return (
        "Command guide:\n"
        "  Natural English examples:\n"
        "    take off to 3 meters, hover for two seconds, and land\n"
        "    fly in a 5 meter radius circle at 3 meter altitude\n"
        "    do a 10 meter square search pattern at 3 meters\n"
        "    go 10 meters north and 5 meters east at 3 meters altitude\n"
        "    hold position\n"
        "    switch mode to guided\n"
        "    switch mode to althold\n"
        "    land now\n"
        "    return to launch\n"
        "  Compact command forms:\n"
        "    takeoff h=3 hover_s=2\n"
        "    circle r=5 h=3 n=36\n"
        "    square size=10 h=3 passes=4\n"
        "    goto x=10 y=5 h=3\n"
        "    mode guided | mode alt_hold\n"
        "    hold | land | rtl\n"
        "  Parameter dictionary:\n"
        "    h / altitude / height: target altitude above launch in meters\n"
        "    r / radius: circle radius in meters\n"
        "    n: number of circle waypoints\n"
        "    x / north: north offset in meters\n"
        "    y / east: east offset in meters\n"
        "    size / side: square side length in meters\n"
        "    passes: lawnmower search passes inside square\n"
        "    hover_s / seconds: hover duration in seconds"
    )


def _normalize_text(text: str) -> str:
    normalized = text.strip().lower()
    normalized = normalized.replace("take-off", "takeoff").replace("take off", "takeoff")
    normalized = normalized.replace("secobds", "seconds").replace("secondes", "seconds")
    for word, value in _NUMBER_WORDS.items():
        normalized = re.sub(rf"\b{word}\b", str(value), normalized)
    return normalized


_NUMBER_WORDS = {
    "zero": 0,
    "one": 1,
    "two": 2,
    "three": 3,
    "four": 4,
    "five": 5,
    "six": 6,
    "seven": 7,
    "eight": 8,
    "nine": 9,
    "ten": 10,
    "eleven": 11,
    "twelve": 12,
    "fifteen": 15,
    "twenty": 20,
    "thirty": 30,
    "forty": 40,
    "fifty": 50,
    "sixty": 60,
}


def _duration_seconds(text: str) -> Optional[float]:
    keyed = _parse_key_values(text)
    for name in ("hover_s", "seconds", "duration", "wait"):
        if name in keyed:
            return keyed[name]
    patterns = (
        rf"\b(?:hover|hold|wait|pause)\s*(?:for)?\s*([-+]?\d+(?:\.\d+)?)\s*{TIME_UNIT_PATTERN}\b",
        rf"\b([-+]?\d+(?:\.\d+)?)\s*{TIME_UNIT_PATTERN}\b",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return float(match.group(1))
    return None


def _takeoff_altitude(text: str, default_altitude_m: float) -> float:
    altitude = _named_distance(text, ("altitude", "height", "alt", "h"))
    if altitude is not None:
        return altitude
    match = re.search(
        rf"\b(?:takeoff|lift off|climb)\s*(?:to)?\s*([-+]?\d+(?:\.\d+)?)\s*{DISTANCE_UNIT_PATTERN}\b",
        text,
    )
    if match:
        return float(match.group(1))
    return default_altitude_m


def _named_distance(text: str, names: tuple[str, ...]) -> Optional[float]:
    for name in names:
        patterns = (
            rf"\b{name}\s*(?:=|is|of|to)?\s*([-+]?\d+(?:\.\d+)?)\s*{DISTANCE_UNIT_PATTERN}\b",
            rf"\b([-+]?\d+(?:\.\d+)?)\s*{DISTANCE_UNIT_PATTERN}\s+{name}\b",
        )
        for pattern in patterns:
            match = re.search(pattern, text)
            if match:
                return float(match.group(1))
    return None


def _numbers(text: str) -> list[float]:
    return [
        float(match.group(1))
        for match in re.finditer(
            rf"([-+]?\d+(?:\.\d+)?)\s*{DISTANCE_UNIT_PATTERN}\b",
            text,
        )
    ]


def _direction_distance(text: str, direction: str) -> Optional[float]:
    patterns = (
        rf"\b([-+]?\d+(?:\.\d+)?)\s*{DISTANCE_UNIT_PATTERN}\s+{direction}\b",
        rf"\b{direction}\s*([-+]?\d+(?:\.\d+)?)\s*{DISTANCE_UNIT_PATTERN}\b",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return float(match.group(1))
    return None


def _first_or(default: float, values: list[float]) -> float:
    return values[0] if values else default


def _first_param(params: dict[str, float], names: tuple[str, ...], default: float) -> float:
    for name in names:
        if name in params:
            return params[name]
    return default


def _positive(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive finite number")
    return value
