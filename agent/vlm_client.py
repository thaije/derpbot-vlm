import base64
import io
import json
import logging
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional

from pydantic import BaseModel, Field

logger = logging.getLogger(__name__)

# The VLM "brain" (prompts + decision schema + inference params) lives in
# shared/ so the Python sim agent and the Android robot app (issue #19) read ONE
# source of truth and never drift. Edit prompts in shared/prompts/*.txt and
# enums/params in shared/vlm_schema.json — do NOT inline them back here.
_SHARED_DIR = Path(__file__).resolve().parent.parent / "shared"

with open(_SHARED_DIR / "vlm_schema.json", encoding="utf-8") as _f:
    _SCHEMA = json.load(_f)


def _load_prompt(name: str) -> str:
    """Load a shared prompt verbatim (no stripping → byte-identical to source)."""
    return (_SHARED_DIR / "prompts" / name).read_text(encoding="utf-8")


MAX_IMAGE_DIM = _SCHEMA["image"]["max_dim"]
JPEG_QUALITY = _SCHEMA["image"]["jpeg_quality"]
DETECTION_TEMPERATURE = _SCHEMA["inference"]["detection_temperature"]
VERIFICATION_TEMPERATURE = _SCHEMA["inference"]["verification_temperature"]
_DIST_MIN = _SCHEMA["navigation_decision"]["drive_distance_m"]["min"]
_DIST_MAX = _SCHEMA["navigation_decision"]["drive_distance_m"]["max"]
_LOCATION_VALUES = _SCHEMA["navigation_decision"]["location_values"]
_HEADING_TOKENS = set(_SCHEMA["navigation_decision"]["heading_values"])
_TURN_ANGLE_VALUES = set(_SCHEMA["navigation_decision"]["turn_angle_deg_values"])

SYSTEM_PROMPT = _load_prompt("detection_system.txt")


class NavigationDecision(BaseModel):
    target_visible: bool
    target_location: Optional[str] = None
    heading: Literal["left", "center", "right"]
    turn_angle_deg: int = 0
    drive_distance_m: float = Field(ge=_DIST_MIN, le=_DIST_MAX)
    reason: str


NAV_DECISION_SCHEMA = NavigationDecision.model_json_schema()


# ── Verifier (skeptical second call, #10) ───────────────────────────────────
# Given a tightly cropped image of a candidate region, decide whether it
# really shows the target. Deliberately framed in the OPPOSITE direction from
# the detection prompt (which rewards aggressive "see anything that fits"
# behaviour). The verifier is asked to enumerate evidence both for AND against
# so the model commits to a calibrated judgement instead of agreeing reflexively.

VERIFIER_SYSTEM_PROMPT = _load_prompt("verifier_system.txt")


class VerificationDecision(BaseModel):
    confirmed: bool
    matches: list[str] = []
    mismatches: list[str] = []
    reason: str = ""


VERIFY_DECISION_SCHEMA = VerificationDecision.model_json_schema()


@dataclass
class VerifyResult:
    confirmed: bool
    matches: list[str]
    mismatches: list[str]
    reason: str


def _parse_verify_response(raw: str) -> Optional[VerifyResult]:
    """Tolerant parse of a verifier reply. Strict JSON → fenced → embedded →
    heuristic last resort. Mirrors `_parse_vlm_response`."""
    if not raw:
        return None

    def _from_dict(d: dict) -> VerifyResult:
        return VerifyResult(
            confirmed=bool(d.get("confirmed", False)),
            matches=list(d.get("matches") or []),
            mismatches=list(d.get("mismatches") or []),
            reason=str(d.get("reason", raw[:200])),
        )

    s = raw.strip()
    if s.startswith("{"):
        try:
            return _from_dict(json.loads(s))
        except (json.JSONDecodeError, ValueError):
            pass

    fence = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", raw, re.DOTALL | re.IGNORECASE)
    if fence:
        try:
            return _from_dict(json.loads(fence.group(1)))
        except (json.JSONDecodeError, ValueError):
            pass

    obj = re.search(r"\{[^{}]*\"confirmed\"\s*:\s*(?:true|false)[^{}]*\}", raw, re.IGNORECASE)
    if obj:
        try:
            return _from_dict(json.loads(obj.group(0)))
        except (json.JSONDecodeError, ValueError):
            pass

    text = raw.lower()
    confirmed = bool(re.search(r"confirmed\s*[:=]\s*true", text)) or bool(
        re.search(r"\b(yes|confirmed|correct)\b.*\btarget\b", text)
    )
    logger.warning("Verifier response unstructured, heuristic parse: %s", raw[:200])
    return VerifyResult(
        confirmed=confirmed,
        matches=[],
        mismatches=[],
        reason=raw[:200],
    )


@dataclass
class VLMResult:
    target_visible: bool
    heading: str
    drive_distance_m: float
    target_location: Optional[str]
    reason: str
    turn_angle_deg: int = 0
    image_width: int = 0
    image_height: int = 0


def _clamp_distance(v) -> float:
    try:
        d = float(v)
    except (TypeError, ValueError):
        return _DIST_MIN
    return max(_DIST_MIN, min(_DIST_MAX, d))


def _coerce_heading(v) -> str:
    s = str(v or "").strip().lower()
    if s in _HEADING_TOKENS:
        return s
    if s in {"l", "ccw", "anticlockwise"}:
        return "left"
    if s in {"r", "cw", "clockwise"}:
        return "right"
    return "center"


def _coerce_location(v) -> Optional[str]:
    if v is None:
        return None
    s = str(v).strip().lower()
    if s in _LOCATION_VALUES:
        return s
    return None


def _coerce_turn_angle(v) -> int:
    """Clamp turn_angle_deg to the allowed set {-90,-60,-30,0,30,60,90}."""
    try:
        deg = int(float(v))
    except (TypeError, ValueError):
        return 0
    # snap to nearest allowed value
    return min(_TURN_ANGLE_VALUES, key=lambda a: abs(a - deg))


def _heading_from_turn_angle(deg: int) -> str:
    if deg < 0:
        return "left"
    if deg > 0:
        return "right"
    return "center"


def _result_from_dict(d: dict, raw: str) -> VLMResult:
    turn_deg = _coerce_turn_angle(d.get("turn_angle_deg", 0))
    heading = _coerce_heading(d.get("heading"))
    # If heading is missing/center but turn_angle_deg is set, derive heading
    if heading == "center" and turn_deg != 0:
        heading = _heading_from_turn_angle(turn_deg)
    # If turn_angle_deg is 0 but heading is set, derive angle from heading
    if turn_deg == 0 and heading != "center":
        turn_deg = -30 if heading == "left" else 30 if heading == "right" else 0
    return VLMResult(
        target_visible=bool(d.get("target_visible", False)),
        heading=heading,
        turn_angle_deg=turn_deg,
        drive_distance_m=_clamp_distance(d.get("drive_distance_m", 0.0)),
        target_location=_coerce_location(d.get("target_location")),
        reason=str(d.get("reason", d.get("reasoning", raw[:200]))),
    )


def _parse_vlm_response(raw: str) -> Optional[VLMResult]:
    if not raw:
        return None

    s = raw.strip()
    if s.startswith("{"):
        try:
            return _result_from_dict(json.loads(s), raw)
        except (json.JSONDecodeError, ValueError):
            pass

    fence = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", raw, re.DOTALL | re.IGNORECASE)
    if fence:
        try:
            return _result_from_dict(json.loads(fence.group(1)), raw)
        except (json.JSONDecodeError, ValueError):
            pass

    obj = re.search(r"\{[^{}]*\"target_visible\"\s*:\s*(?:true|false)[^{}]*\}", raw, re.IGNORECASE)
    if obj:
        try:
            return _result_from_dict(json.loads(obj.group(0)), raw)
        except (json.JSONDecodeError, ValueError):
            pass

    text = raw.lower()
    visible = bool(re.search(r"target_visible\s*[:=]\s*true", text)) or bool(
        re.search(r"i\s+(?:can\s+)?see\s+(?:a\s+|the\s+)?(?:fire\s+extinguisher|drink|drill|pipe|suitcase|target)", text)
    )
    heading = "center"
    turn_deg = 0
    # Try turn_angle_deg first
    m = re.search(r"turn_angle_deg\s*[:=]\s*(-?\d+)", text)
    if m:
        turn_deg = _coerce_turn_angle(m.group(1))
        heading = _heading_from_turn_angle(turn_deg)
    else:
        m = re.search(r"heading\s*[:=]\s*\"?(left|center|right)\"?", text)
        if m:
            heading = m.group(1)
        elif re.search(r"\bturn\s+left|go\s+left\b", text):
            heading = "left"
        elif re.search(r"\bturn\s+right|go\s+right\b", text):
            heading = "right"
        turn_deg = -30 if heading == "left" else 30 if heading == "right" else 0

    dist = 0.5
    m = re.search(r"(?:drive_distance_m|distance|drive)\s*[:=]\s*([0-9]*\.?[0-9]+)", text)
    if m:
        dist = _clamp_distance(m.group(1))

    logger.warning("VLM response unstructured, heuristic parse: %s", raw[:200])
    return VLMResult(
        target_visible=visible,
        heading=heading,
        turn_angle_deg=turn_deg,
        drive_distance_m=dist,
        target_location=None,
        reason=raw[:200],
    )


class VLMClient:
    def __init__(self, config: dict):
        model_cfg = config["model"]
        self.model_name = model_cfg["name"]
        self.backend = model_cfg.get("backend", "ollama")
        self.is_cloud = self.backend == "ollama-cloud"

        inf_cfg = config["inference"]
        self.max_retries = inf_cfg.get("max_retries", 3)
        self.timeout = inf_cfg.get("timeout_s", 30.0)
        self._client = None

    def start(self, ready_timeout: float = 300.0):
        from ollama import Client
        import httpx
        self._client = Client()
        # Preserve the auto-configured base_url (e.g. http://127.0.0.1:11434)
        # but set a timeout — the default is None (wait forever), which blocks
        # the agent if the cloud VLM hangs (#17).
        base_url = self._client._client._base_url
        self._client._client = httpx.Client(timeout=self.timeout, base_url=base_url)

        if self.is_cloud:
            logger.info("Cloud model %s — skipping local pre-load", self.model_name)
            try:
                self._client.chat(
                    model=self.model_name,
                    messages=[{"role": "user", "content": "ping"}],
                    keep_alive=0,
                )
                logger.info("Cloud model %s reachable", self.model_name)
            except Exception as e:
                logger.warning("Cloud model reachability check: %s", e)
            return

        logger.info("Pre-loading Ollama model %s...", self.model_name)
        import time
        t0 = time.time()
        try:
            self._client.chat(
                model=self.model_name,
                messages=[{"role": "user", "content": "ping"}],
                keep_alive=-1,
            )
        except Exception as e:
            logger.warning("Model pre-load attempt: %s (model may still be downloading)", e)
        logger.info("Ollama model %s ready (%.1fs)", self.model_name, time.time() - t0)

    def stop(self):
        if self._client:
            try:
                self._client.chat(model=self.model_name, messages=[], keep_alive=0)
            except Exception:
                pass
        logger.info("Ollama model unloaded")

    def query(self, image, prompt: str, verbose: bool = False) -> Optional[VLMResult]:
        if self._client is None:
            raise RuntimeError("VLM client not started")

        w, h = image.size
        if max(w, h) > MAX_IMAGE_DIM:
            scale = MAX_IMAGE_DIM / max(w, h)
            image = image.resize((int(w * scale), int(h * scale)))
        sent_w, sent_h = image.size

        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=JPEG_QUALITY)
        img_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        if verbose:
            # Full prompt I/O for the debug harness (#13). Production callers
            # leave verbose=False and keep the terse one-line summary below.
            logger.info("VLM REQUEST · system prompt:\n%s", SYSTEM_PROMPT)
            logger.info("VLM REQUEST · user prompt:\n%s", prompt)
            logger.info("VLM REQUEST · image sent %dx%d JPEG q%d",
                        sent_w, sent_h, JPEG_QUALITY)

        for attempt in range(1, self.max_retries + 1):
            try:
                keep_alive = 0 if self.is_cloud else -1
                response = self._client.chat(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": prompt, "images": [img_b64]},
                    ],
                    format=NAV_DECISION_SCHEMA,
                    options={"temperature": DETECTION_TEMPERATURE},
                    stream=False,
                    keep_alive=keep_alive,
                )
                raw = response.message.content
                if not raw:
                    logger.warning("VLM returned empty (attempt %d/%d)", attempt, self.max_retries)
                    continue
                if verbose:
                    logger.info("VLM RAW RESPONSE:\n%s", raw)

                result = _parse_vlm_response(raw)
                if result is not None:
                    result.image_width = sent_w
                    result.image_height = sent_h
                    logger.info("VLM: vis=%s hdg=%s turn=%+d° dist=%.2f loc=%s img=%dx%d | %s",
                                result.target_visible, result.heading,
                                result.turn_angle_deg,
                                result.drive_distance_m, result.target_location,
                                sent_w, sent_h, result.reason[:100])
                    if result.target_visible and result.target_location is None:
                        logger.warning("VLM raw (visible w/o location): %.500s", raw)
                    return result
                logger.warning("VLM response unparseable (attempt %d/%d): %.200s", attempt, self.max_retries, raw)

            except Exception as e:
                logger.error("VLM query error (attempt %d/%d): %s", attempt, self.max_retries, e)

        logger.error("All %d VLM attempts failed, defaulting to stop", self.max_retries)
        return VLMResult(
            target_visible=False,
            heading="center",
            drive_distance_m=0.0,
            target_location=None,
            reason="VLM query failed after retries",
            image_width=sent_w,
            image_height=sent_h,
        )

    def verify_candidate(self, image, target_name: str, location: str = "",
                         verbose: bool = False) -> Optional[VerifyResult]:
        """Skeptical second call on the full camera image (#14).

        Sends the full image with a location hint instead of a cropped bbox.
        Returns the verifier's judgement, or None on repeated failure.
        """
        if self._client is None:
            raise RuntimeError("VLM client not started")

        w, h = image.size
        if max(w, h) > MAX_IMAGE_DIM:
            scale = MAX_IMAGE_DIM / max(w, h)
            image = image.resize((int(w * scale), int(h * scale)))
        sent_w, sent_h = image.size

        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=JPEG_QUALITY)
        img_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        target_natural = (target_name or "the target").replace("_", " ")
        location_text = f" at the {location}" if location else ""
        user_prompt = (
            f"Target: {target_name} (natural language: \"{target_natural}\")\n"
            f"The detector flagged a possible target{location_text} in this image."
            " Confirm or reject.\n"
            "Be strict; reject vaguely similar shapes."
        )

        if verbose:
            logger.info("VERIFIER REQUEST · system prompt:\n%s", VERIFIER_SYSTEM_PROMPT)
            logger.info("VERIFIER REQUEST · user prompt:\n%s", user_prompt)
            logger.info("VERIFIER REQUEST · image sent %dx%d JPEG q%d",
                        sent_w, sent_h, JPEG_QUALITY)

        for attempt in range(1, self.max_retries + 1):
            try:
                keep_alive = 0 if self.is_cloud else -1
                response = self._client.chat(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": VERIFIER_SYSTEM_PROMPT},
                        {"role": "user", "content": user_prompt, "images": [img_b64]},
                    ],
                    format=VERIFY_DECISION_SCHEMA,
                    options={"temperature": VERIFICATION_TEMPERATURE},  # lower temp → more deterministic verdicts
                    stream=False,
                    keep_alive=keep_alive,
                )
                raw = response.message.content
                if not raw:
                    logger.warning("Verifier returned empty (attempt %d/%d)", attempt, self.max_retries)
                    continue
                if verbose:
                    logger.info("VERIFIER RAW RESPONSE:\n%s", raw)

                result = _parse_verify_response(raw)
                if result is not None:
                    logger.info(
                        "VERIFY: confirmed=%s img=%dx%d matches=%d mismatches=%d | %s",
                        result.confirmed, sent_w, sent_h,
                        len(result.matches), len(result.mismatches),
                        result.reason[:120],
                    )
                    return result
                logger.warning("Verifier response unparseable (attempt %d/%d): %.200s", attempt, self.max_retries, raw)

            except Exception as e:
                logger.error("Verifier query error (attempt %d/%d): %s", attempt, self.max_retries, e)

        logger.error("All %d verifier attempts failed; defaulting to REJECT (safe)", self.max_retries)
        return VerifyResult(confirmed=False, matches=[], mismatches=[], reason="verifier failed")
