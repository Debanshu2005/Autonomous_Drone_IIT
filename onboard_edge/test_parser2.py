import sys
from trajectory_engine import parse_mission
print("takeoff:", parse_mission("takeoff at an height of 5m."))
print("arm:", parse_mission("arm"))
