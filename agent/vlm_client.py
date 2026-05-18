import base64
import io
import logging
from dataclasses import dataclass
from typing import Literal, Optional

from pydantic import BaseModel

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = (
    "You navigate a mobile robot through indoor environments using camera images. "
    "Your goal is to explore efficiently and find the mission target object. "
    "\n\nActions: forward (move ahead), backward (reverse), left (rotate left ~30deg), "
    "right (rotate right ~30deg), stop (halt completely)."
    "\n\nCRITICAL RULES:"
    "\n- target_visible MUST be false unless you can CLEARLY see the named target object in the image."
    "\n- Walls, doors, furniture, and other objects are NOT the target. Only set target_visible=true "
    "when you see the specific target object named in the mission."
    "\n- If you have been turning in one direction for several steps, turn the opposite direction next. "
    "Alternate turning to explore the full environment."
    "\n- Prioritize forward movement when the path ahead is clear. Only turn to avoid obstacles or "
    "search new areas."
    "\n- Do not repeat the same action many times in a row. Vary your actions."
)


class NavigationAction(BaseModel):
    action: Literal["forward", "backward", "left", "right", "stop"]
    reasoning: str
    target_visible: bool


NAV_ACTION_SCHEMA = NavigationAction.model_json_schema()


@dataclass
class VLMResult:
    action: str
    reasoning: str
    target_visible: bool


class VLMClient:
    def __init__(self, config: dict):
        model_cfg = config["model"]
        self.model_name = model_cfg["name"]

        inf_cfg = config["inference"]
        self.max_retries = inf_cfg.get("max_retries", 3)
        self.timeout = inf_cfg.get("timeout_s", 30.0)
        self._client = None

    def start(self, ready_timeout: float = 300.0):
        from ollama import Client
        self._client = Client()
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

    def query(self, image, prompt: str) -> Optional[VLMResult]:
        if self._client is None:
            raise RuntimeError("VLM client not started")

        max_dim = 384
        w, h = image.size
        if max(w, h) > max_dim:
            scale = max_dim / max(w, h)
            image = image.resize((int(w * scale), int(h * scale)))

        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=70)
        img_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        for attempt in range(1, self.max_retries + 1):
            try:
                response = self._client.chat(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": prompt, "images": [img_b64]},
                    ],
                    format=NAV_ACTION_SCHEMA,
                    options={"temperature": 0.3},
                    stream=False,
                    keep_alive=-1,
                )
                raw = response.message.content
                if not raw:
                    logger.warning("VLM returned empty (attempt %d/%d)", attempt, self.max_retries)
                    continue

                parsed = NavigationAction.model_validate_json(raw)
                return VLMResult(
                    action=parsed.action,
                    reasoning=parsed.reasoning,
                    target_visible=parsed.target_visible,
                )

            except Exception as e:
                logger.error("VLM query error (attempt %d/%d): %s", attempt, self.max_retries, e)

        logger.error("All %d VLM attempts failed, defaulting to stop", self.max_retries)
        return VLMResult(action="stop", reasoning="VLM query failed after retries", target_visible=False)