import sys
import re
from trajectory_engine import _normalize_text, _remove_mode_request_text, _mode_request

def test(text):
    normalized = _normalize_text(text)
    print("normalized:", normalized)
    
    has_takeoff = bool(re.search(r'\btakeoff\b', normalized))
    print("has_takeoff:", has_takeoff)

test("takeoff at an height of 5m.")
