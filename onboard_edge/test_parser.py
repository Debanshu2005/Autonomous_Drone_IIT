import sys
from trajectory_engine import parse_task_sequence
try:
    print(parse_task_sequence("takeoff at an height of 5m."))
except Exception as e:
    print(f"Error 1: {e}")

try:
    print(parse_task_sequence("arm"))
except Exception as e:
    print(f"Error 2: {e}")
