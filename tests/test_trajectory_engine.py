from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
ONBOARD_EDGE = ROOT / "onboard_edge"
if str(ONBOARD_EDGE) not in sys.path:
    sys.path.insert(0, str(ONBOARD_EDGE))

from sensor_check import NavigationMode
from trajectory_engine import (
    TaskAction,
    TargetFrame,
    VehicleOrigin,
    _square_search_targets,
    build_trajectory,
    parse_task_sequence,
)


class TrajectoryParserTests(unittest.TestCase):
    def test_cardinal_and_body_direction_phrases(self) -> None:
        cases = {
            "go 10m north": (TaskAction.GOTO, {"north": 10.0, "east": 0.0}),
            "go north 10m": (TaskAction.GOTO, {"north": 10.0, "east": 0.0}),
            "fly 2m right": (TaskAction.GOTO, {"north": 0.0, "east": 2.0}),
            "go 2m left": (TaskAction.GOTO, {"north": 0.0, "east": -2.0}),
            "go 5m back": (TaskAction.GOTO, {"north": -5.0, "east": 0.0}),
        }
        for command, (action, expected) in cases.items():
            with self.subTest(command=command):
                task = parse_task_sequence(command).tasks[0]
                self.assertEqual(task.action, action)
                for key, value in expected.items():
                    self.assertEqual(task.params[key], value)

    def test_absolute_and_relative_altitude_phrases(self) -> None:
        absolute = parse_task_sequence("go 5m high").tasks[0]
        self.assertEqual(absolute.action, TaskAction.GOTO)
        self.assertEqual(absolute.params["h"], 5.0)

        relative = parse_task_sequence("move 10m north and 2m up").tasks[0]
        self.assertEqual(relative.action, TaskAction.MOVE_RELATIVE)
        self.assertEqual(relative.params["dn"], 10.0)
        self.assertEqual(relative.params["dd"], -2.0)

        descend = parse_task_sequence("lower 50cm").tasks[0]
        self.assertEqual(descend.action, TaskAction.MOVE_RELATIVE)
        self.assertEqual(descend.params["dd"], 0.5)

        climb = parse_task_sequence("climb 2").tasks[0]
        self.assertEqual(climb.action, TaskAction.MOVE_RELATIVE)
        self.assertEqual(climb.params["dd"], -2.0)

    def test_signed_key_value_goto(self) -> None:
        task = parse_task_sequence("goto x=3 y=-2 h=4").tasks[0]
        self.assertEqual(task.params["north"], 3.0)
        self.assertEqual(task.params["east"], -2.0)
        self.assertEqual(task.params["h"], 4.0)

    def test_shape_uses_named_radius_and_altitude(self) -> None:
        tasks = parse_task_sequence("takeoff to 5m, circle with 3m radius at 4m altitude").tasks
        self.assertEqual(tasks[0].action, TaskAction.TAKEOFF)
        self.assertEqual(tasks[0].params["h"], 5.0)
        self.assertEqual(tasks[1].action, TaskAction.CIRCLE)
        self.assertEqual(tasks[1].params["r"], 3.0)
        self.assertEqual(tasks[1].params["h"], 4.0)

    def test_takeoff_hover_s_adds_one_hover_task(self) -> None:
        tasks = parse_task_sequence("takeoff h=3 hover_s=2").tasks
        self.assertEqual([task.action for task in tasks], [TaskAction.TAKEOFF, TaskAction.HOVER])
        self.assertEqual(tasks[1].params["hover_s"], 2.0)

        tasks = parse_task_sequence("takeoff h=3 hover hover_s=2").tasks
        self.assertEqual([task.action for task in tasks], [TaskAction.TAKEOFF, TaskAction.HOVER])
        self.assertEqual(tasks[1].params["hover_s"], 2.0)

    def test_mode_names_are_normalized(self) -> None:
        tasks = parse_task_sequence("switch mode to alt hold, then rtl").tasks
        self.assertEqual([(task.action, task.notes) for task in tasks], [
            (TaskAction.SET_MODE, ("mode=ALT_HOLD",)),
            (TaskAction.RTL, ()),
        ])

        tasks = parse_task_sequence("mode loiter").tasks
        self.assertEqual([(task.action, task.notes) for task in tasks], [
            (TaskAction.SET_MODE, ("mode=LOITER",)),
        ])

    def test_missing_altitude_keeps_current_altitude(self) -> None:
        report = SimpleNamespace(mode=NavigationMode.MODE_B_LOCAL)
        origin = VehicleOrigin(0.0, 0.0, -5.0, relative_alt_m=5.0)

        circle = parse_task_sequence("circle with 3m radius").tasks[0]
        circle_plan = build_trajectory(circle, report, origin)
        self.assertEqual(circle_plan.target_altitude_m, 5.0)
        self.assertTrue(all(target.down_m == -5.0 for target in circle_plan.local_targets))

        goto = parse_task_sequence("go 10m north").tasks[0]
        goto_plan = build_trajectory(goto, report, origin)
        self.assertEqual(goto_plan.target_altitude_m, 5.0)
        self.assertEqual(goto_plan.local_targets[0].down_m, -5.0)

    def test_bare_go_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "distance or target coordinate"):
            parse_task_sequence("go")

    def test_square_perimeter_starts_from_current_position(self) -> None:
        task = parse_task_sequence("square size=10 h=3").tasks[0]
        targets = _square_search_targets(task, VehicleOrigin(2.0, 3.0, -3.0), 3.0)
        self.assertEqual(
            [(target.north_m, target.east_m, target.down_m) for target in targets],
            [(12.0, 3.0, -3.0), (12.0, 13.0, -3.0), (2.0, 13.0, -3.0), (2.0, 3.0, -3.0)],
        )

    def test_relative_vertical_trajectory_uses_current_altitude(self) -> None:
        task = parse_task_sequence("go 2m up").tasks[0]
        report = SimpleNamespace(mode=NavigationMode.MODE_B_LOCAL)
        origin = VehicleOrigin(1.0, 2.0, -2.0, relative_alt_m=2.0)
        plan = build_trajectory(task, report, origin)
        self.assertEqual(plan.target_altitude_m, 4.0)
        self.assertEqual(plan.local_targets[0].down_m, -4.0)

    def test_advertised_shapes_generate_targets(self) -> None:
        report = SimpleNamespace(mode=NavigationMode.MODE_B_LOCAL)
        origin = VehicleOrigin(0.0, 0.0, -3.0, relative_alt_m=3.0)
        commands = [
            "triangle size=6 h=3",
            "grid size=10 h=3 passes=4",
            "spiral size=10 h=3 turns=3",
            "figure eight size=5 h=3",
        ]
        for command in commands:
            with self.subTest(command=command):
                task = parse_task_sequence(command).tasks[0]
                plan = build_trajectory(task, report, origin)
                self.assertEqual(plan.frame, TargetFrame.LOCAL_NED)
                self.assertGreater(plan.count, 0)


if __name__ == "__main__":
    unittest.main()
