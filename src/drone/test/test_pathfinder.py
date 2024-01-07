from drone.pathfinder import *
import pytest

def test_find_next_step():
        pf = DummyPathfinder(0, (0,0))
        for i in range(len(HARDCODED_WAYPOINT_TUPLES)-1):
            p1 = HARDCODED_WAYPOINT_TUPLES[i]
            p2 = HARDCODED_WAYPOINT_TUPLES[i+1]
            p2_test = pf.find_next_step(p1, None)
            assert pytest.approx(p2_test[0]) == p2[0]
            assert pytest.approx(p2_test[1]) == p2[1]