import pytest
from agent.vlm_client import (
    NavigationDecision,
    VerificationDecision,
    VLMResult,
    VerifyResult,
    _parse_verify_response,
    _parse_vlm_response,
    _coerce_location,
)


class TestNavigationDecision:
    def test_valid_minimal(self):
        raw = '{"target_visible": false, "heading": "center", "drive_distance_m": 1.0, "reason": "open path"}'
        parsed = NavigationDecision.model_validate_json(raw)
        assert parsed.target_visible is False
        assert parsed.heading == "center"
        assert parsed.drive_distance_m == 1.0
        assert parsed.target_location is None

    def test_with_location(self):
        raw = '{"target_visible": true, "target_location": "left", "heading": "left", "drive_distance_m": 0.5, "reason": "target on left"}'
        parsed = NavigationDecision.model_validate_json(raw)
        assert parsed.target_visible is True
        assert parsed.target_location == "left"
        assert parsed.heading == "left"

    def test_all_locations(self):
        for loc in ["far left", "left", "center-left", "center", "center-right", "right", "far right"]:
            raw = f'{{"target_visible": true, "target_location": "{loc}", "heading": "center", "drive_distance_m": 1.0, "reason": "test"}}'
            parsed = NavigationDecision.model_validate_json(raw)
            assert parsed.target_location == loc

    def test_distance_bounds(self):
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json(
                '{"target_visible": false, "heading": "center", "drive_distance_m": 3.0, "reason": "x"}')
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json(
                '{"target_visible": false, "heading": "center", "drive_distance_m": -0.1, "reason": "x"}')

    def test_heading_must_be_literal(self):
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json(
                '{"target_visible": false, "heading": "forward", "drive_distance_m": 1.0, "reason": "x"}')

    def test_missing_field_rejected(self):
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json('{"target_visible": false, "heading": "center"}')


class TestVLMResult:
    def test_fields(self):
        r = VLMResult(target_visible=True, heading="right", drive_distance_m=1.2,
                      target_location="center-right", reason="ok")
        assert r.target_visible is True
        assert r.heading == "right"
        assert r.drive_distance_m == 1.2
        assert r.target_location == "center-right"


class TestCoerceLocation:
    def test_valid_locations(self):
        assert _coerce_location("left") == "left"
        assert _coerce_location("center") == "center"
        assert _coerce_location("far left") == "far left"
        assert _coerce_location("center-right") == "center-right"

    def test_none(self):
        assert _coerce_location(None) is None

    def test_invalid(self):
        assert _coerce_location("above") is None
        assert _coerce_location("") is None


class TestParseVLMResponse:
    def test_clean_json(self):
        r = _parse_vlm_response(
            '{"target_visible": true, "target_location": "left", "heading": "center", "drive_distance_m": 1.5, "reason": "ok"}')
        assert r.target_visible is True
        assert r.heading == "center"
        assert r.drive_distance_m == 1.5
        assert r.target_location == "left"

    def test_json_in_code_fence(self):
        raw = '```json\n{"target_visible": false, "target_location": null, "heading": "left", "drive_distance_m": 0.5, "reason": "wall"}\n```'
        r = _parse_vlm_response(raw)
        assert r.target_visible is False
        assert r.heading == "left"
        assert r.drive_distance_m == 0.5

    def test_distance_clamp_above(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "center", "drive_distance_m": 5.0, "reason": "x"}')
        assert r.drive_distance_m == 2.0

    def test_distance_clamp_below(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "center", "drive_distance_m": -2.0, "reason": "x"}')
        assert r.drive_distance_m == 0.0

    def test_heading_coercion_uppercase(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "LEFT", "drive_distance_m": 0.8, "reason": "x"}')
        assert r.heading == "left"

    def test_heading_unknown_defaults_center(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "forward", "drive_distance_m": 0.8, "reason": "x"}')
        assert r.heading == "center"

    def test_free_text_visible(self):
        r = _parse_vlm_response("I can see the target fire extinguisher on the wall to the left, turn left.")
        assert r.target_visible is True
        assert r.heading == "left"

    def test_free_text_not_visible(self):
        r = _parse_vlm_response("The fire extinguisher is not visible in this image. Go right.")
        assert r.target_visible is False
        assert r.heading == "right"

    def test_empty_returns_none(self):
        assert _parse_vlm_response("") is None

    def test_unparseable_defaults_safe(self):
        r = _parse_vlm_response("The image shows a corridor with walls and a door.")
        assert r.target_visible is False
        assert r.heading == "center"
        assert r.drive_distance_m == 0.5
        assert r.target_location is None


class TestVerificationDecision:
    def test_valid_confirmed(self):
        raw = '{"confirmed": true, "matches": ["red cylinder"], "mismatches": [], "reason": "clear fire extinguisher"}'
        v = VerificationDecision.model_validate_json(raw)
        assert v.confirmed is True
        assert v.matches == ["red cylinder"]
        assert v.mismatches == []

    def test_valid_rejected(self):
        raw = '{"confirmed": false, "matches": [], "mismatches": ["wrong colour", "rectangular"], "reason": "looks like a brick"}'
        v = VerificationDecision.model_validate_json(raw)
        assert v.confirmed is False
        assert len(v.mismatches) == 2


class TestParseVerifyResponse:
    def test_clean_json_confirmed(self):
        r = _parse_verify_response('{"confirmed": true, "matches": ["red", "narrow"], "mismatches": [], "reason": "ok"}')
        assert r.confirmed is True
        assert r.matches == ["red", "narrow"]

    def test_clean_json_rejected(self):
        r = _parse_verify_response('{"confirmed": false, "matches": [], "mismatches": ["wall edge"], "reason": "not target"}')
        assert r.confirmed is False
        assert r.mismatches == ["wall edge"]

    def test_json_in_code_fence(self):
        raw = '```json\n{"confirmed": false, "matches": [], "mismatches": ["floor seam"], "reason": "no"}\n```'
        r = _parse_verify_response(raw)
        assert r.confirmed is False

    def test_missing_fields_defaults(self):
        r = _parse_verify_response('{"confirmed": true, "reason": "ok"}')
        assert r.confirmed is True
        assert r.matches == []
        assert r.mismatches == []

    def test_freetext_yes_heuristic(self):
        r = _parse_verify_response("Yes, this is clearly the target pipe.")
        assert r.confirmed is True

    def test_freetext_no_defaults_reject(self):
        r = _parse_verify_response("This appears to be a brick wall, not the target.")
        assert r.confirmed is False

    def test_empty_returns_none(self):
        assert _parse_verify_response("") is None