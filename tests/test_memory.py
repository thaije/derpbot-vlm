import math
import pytest
from agent.memory import VisitedCells


def test_mark_and_count():
    m = VisitedCells(cell_size_m=0.5)
    m.mark(0.0, 0.0)
    m.mark(0.1, 0.1)  # same cell
    m.mark(0.6, 0.0)  # different cell
    assert m.count() == 2


def test_is_visited_matches_mark():
    m = VisitedCells(cell_size_m=0.5)
    m.mark(1.0, 2.0)
    assert m.is_visited(1.1, 2.2)
    assert not m.is_visited(3.0, 3.0)


def test_heading_summary_unexplored_when_empty():
    m = VisitedCells()
    s = m.heading_summary(0.0, 0.0, 0.0)
    assert s["left"] == 0.0
    assert s["center"] == 0.0
    assert s["right"] == 0.0


def test_heading_summary_visited_center():
    m = VisitedCells(cell_size_m=0.5)
    # Mark cells along positive x at 0.5, 1.0, 1.5, 2.0 m
    for d in (0.5, 1.0, 1.5, 2.0):
        m.mark(d, 0.0)
    s = m.heading_summary(0.0, 0.0, 0.0)
    assert s["center"] == pytest.approx(1.0, abs=0.01)
    assert s["left"] < 0.5
    assert s["right"] < 0.5


def test_render_summary_contains_all_headings():
    m = VisitedCells()
    summary = m.render_prompt_summary(0.0, 0.0, 0.0)
    for hdg in ("left", "center", "right"):
        assert hdg in summary
