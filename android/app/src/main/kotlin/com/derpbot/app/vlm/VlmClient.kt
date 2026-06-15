package com.derpbot.app.vlm

import android.content.Context
import android.graphics.Bitmap
import android.util.Base64
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.util.concurrent.TimeUnit
import kotlin.math.max
import kotlin.math.min

/**
 * Kotlin port of agent/vlm_client.py for the Android RVR stack (issue #19).
 *
 * Loads prompts and schema from `assets/` (bundled verbatim from shared/ by the
 * build — one source of truth). Posts to the Ollama /api/chat endpoint over WiFi.
 * Response parsing mirrors the Python tolerant chain: strict JSON → fenced →
 * embedded object → heuristic.
 *
 * Call [query] on a background dispatcher (it blocks on HTTP).
 * [verify] is the skeptical second call matching the Python verifier path.
 */
class VlmClient(
    context: Context,
    private val ollamaBaseUrl: String,   // direct cloud "https://ollama.com" or local daemon "http://192.168.x.x:11434"
    private val modelName: String,
    private val apiKey: String? = null,  // set for direct cloud (Authorization: Bearer …); null for a local daemon
    private val maxRetries: Int = 3,
) {
    private val assets = context.assets

    // Loaded lazily from assets so the constructor is cheap.
    private val schema by lazy {
        JSONObject(assets.open("vlm_schema.json").reader().readText())
    }
    private val maxDim by lazy { schema.getJSONObject("image").getInt("max_dim") }
    private val jpegQuality by lazy { schema.getJSONObject("image").getInt("jpeg_quality") }
    private val detectionTemp by lazy { schema.getJSONObject("inference").getDouble("detection_temperature") }
    private val verifyTemp by lazy { schema.getJSONObject("inference").getDouble("verification_temperature") }

    private val systemPrompt by lazy {
        assets.open("prompts/detection_system.txt").reader().readText()
    }
    private val verifierPrompt by lazy {
        assets.open("prompts/verifier_system.txt").reader().readText()
    }

    private val http = OkHttpClient.Builder()
        .connectTimeout(10, TimeUnit.SECONDS)
        .readTimeout(60, TimeUnit.SECONDS)
        .writeTimeout(30, TimeUnit.SECONDS)
        .build()

    private val chatUrl = "$ollamaBaseUrl/api/chat"
    private val jsonMedia = "application/json; charset=utf-8".toMediaType()

    // ── Public API ────────────────────────────────────────────────────────────

    suspend fun query(bitmap: Bitmap, userPrompt: String): VlmResult = withContext(Dispatchers.IO) {
        val imgB64 = encodeImage(bitmap)
        repeat(maxRetries) { attempt ->
            try {
                val raw = postChat(buildMessages(systemPrompt, userPrompt, imgB64), NAV_SCHEMA, detectionTemp)
                parseVlmResponse(raw)?.let { return@withContext it }
                Log.w(TAG, "VLM unparseable (attempt ${attempt + 1}/$maxRetries): ${raw.take(200)}")
            } catch (e: Exception) {
                Log.e(TAG, "VLM error (attempt ${attempt + 1}/$maxRetries)", e)
            }
        }
        Log.e(TAG, "All $maxRetries VLM attempts failed; defaulting to stop")
        VlmResult(targetVisible = false, heading = "center", driveDistanceM = 0f, targetLocation = null, reason = "VLM failed")
    }

    suspend fun verify(bitmap: Bitmap, targetName: String, location: String = ""): VerifyResult = withContext(Dispatchers.IO) {
        val imgB64 = encodeImage(bitmap)
        val natural = targetName.replace('_', ' ')
        val locText = if (location.isNotEmpty()) " at the $location" else ""
        val userPrompt = "Target: $targetName (natural language: \"$natural\")\n" +
            "The detector flagged a possible target$locText in this image. Confirm or reject.\n" +
            "Be strict; reject vaguely similar shapes."

        repeat(maxRetries) { attempt ->
            try {
                val raw = postChat(buildMessages(verifierPrompt, userPrompt, imgB64), VERIFY_SCHEMA, verifyTemp)
                parseVerifyResponse(raw)?.let { return@withContext it }
                Log.w(TAG, "Verifier unparseable (attempt ${attempt + 1}/$maxRetries): ${raw.take(200)}")
            } catch (e: Exception) {
                Log.e(TAG, "Verifier error (attempt ${attempt + 1}/$maxRetries)", e)
            }
        }
        Log.e(TAG, "All $maxRetries verifier attempts failed; REJECT")
        VerifyResult(confirmed = false, reason = "verifier failed")
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private fun encodeImage(src: Bitmap): String {
        val w = src.width; val h = src.height
        val scaled = if (max(w, h) > maxDim) {
            val s = maxDim.toFloat() / max(w, h)
            Bitmap.createScaledBitmap(src, (w * s).toInt(), (h * s).toInt(), true)
        } else src
        val buf = ByteArrayOutputStream()
        scaled.compress(Bitmap.CompressFormat.JPEG, jpegQuality, buf)
        return Base64.encodeToString(buf.toByteArray(), Base64.NO_WRAP)
    }

    private fun buildMessages(system: String, user: String, imgB64: String) = JSONArray().apply {
        put(JSONObject().put("role", "system").put("content", system))
        put(JSONObject().put("role", "user").put("content", user)
            .put("images", JSONArray().put(imgB64)))
    }

    private fun postChat(messages: JSONArray, format: JSONObject, temperature: Double): String {
        val body = JSONObject()
            .put("model", modelName)
            .put("messages", messages)
            .put("format", format)
            .put("options", JSONObject().put("temperature", temperature))
            .put("stream", false)
            .toString()
        val req = Request.Builder().url(chatUrl).post(body.toRequestBody(jsonMedia))
            .apply { if (!apiKey.isNullOrBlank()) header("Authorization", "Bearer $apiKey") }
            .build()
        http.newCall(req).execute().use { resp ->
            if (!resp.isSuccessful) throw RuntimeException("Ollama HTTP ${resp.code}")
            val respBody = resp.body?.string() ?: throw RuntimeException("Empty response")
            return JSONObject(respBody).getJSONObject("message").getString("content")
        }
    }

    private fun parseVlmResponse(raw: String): VlmResult? {
        if (raw.isBlank()) return null
        fun fromObj(obj: JSONObject): VlmResult {
            val vis = obj.optBoolean("target_visible", false)
            val hdg = coerceHeading(obj.optString("heading", "center"))
            val dist = clampDist(obj.optDouble("drive_distance_m", 0.0))
            val loc = coerceLoc(obj.optString("target_location", ""))
            if (vis && loc == null) Log.w(TAG, "VLM: visible=true but no valid location")
            return VlmResult(vis, hdg, dist, loc, obj.optString("reason", raw.take(200)))
        }
        runCatching { fromObj(JSONObject(raw.trim())) }.onSuccess { return it }
        Regex("""```(?:json)?\s*(\{.*?\})\s*```""", setOf(RegexOption.DOT_MATCHES_ALL, RegexOption.IGNORE_CASE))
            .find(raw)?.groupValues?.getOrNull(1)
            ?.let { runCatching { fromObj(JSONObject(it)) }.onSuccess { r -> return r } }
        Regex("""\{[^{}]*"target_visible"\s*:\s*(?:true|false)[^{}]*\}""", RegexOption.IGNORE_CASE)
            .find(raw)?.value
            ?.let { runCatching { fromObj(JSONObject(it)) }.onSuccess { r -> return r } }
        // heuristic
        val t = raw.lowercase()
        val vis = """"target_visible": true""" in t || """"target_visible":true""" in t
        val hdg = Regex("""heading[":\s]*"?(left|center|right)"?""").find(t)?.groupValues?.getOrNull(1) ?: "center"
        val dist = Regex("""drive_distance_m[":\s]*([0-9]*\.?[0-9]+)""").find(t)?.groupValues?.getOrNull(1)?.toDoubleOrNull()?.let { clampDist(it) } ?: 0.5f
        Log.w(TAG, "VLM heuristic parse: ${raw.take(200)}")
        return VlmResult(vis, hdg, dist, null, raw.take(200))
    }

    private fun parseVerifyResponse(raw: String): VerifyResult? {
        if (raw.isBlank()) return null
        fun fromObj(obj: JSONObject): VerifyResult {
            val arr = { key: String -> obj.optJSONArray(key)?.let { a -> (0 until a.length()).map { a.getString(it) } } ?: emptyList() }
            return VerifyResult(obj.optBoolean("confirmed", false), arr("matches"), arr("mismatches"), obj.optString("reason", ""))
        }
        runCatching { fromObj(JSONObject(raw.trim())) }.onSuccess { return it }
        Regex("""```(?:json)?\s*(\{.*?\})\s*```""", setOf(RegexOption.DOT_MATCHES_ALL, RegexOption.IGNORE_CASE))
            .find(raw)?.groupValues?.getOrNull(1)
            ?.let { runCatching { fromObj(JSONObject(it)) }.onSuccess { r -> return r } }
        Regex("""\{[^{}]*"confirmed"\s*:\s*(?:true|false)[^{}]*\}""", RegexOption.IGNORE_CASE)
            .find(raw)?.value
            ?.let { runCatching { fromObj(JSONObject(it)) }.onSuccess { r -> return r } }
        val t = raw.lowercase()
        val confirmed = Regex("""confirmed\s*[":=]\s*true""").containsMatchIn(t) ||
            Regex("""\b(yes|confirmed|correct)\b.*\btarget\b""").containsMatchIn(t)
        Log.w(TAG, "Verifier heuristic parse: ${raw.take(200)}")
        return VerifyResult(confirmed = confirmed, reason = raw.take(200))
    }

    companion object {
        private const val TAG = "VlmClient"

        private fun clampDist(v: Double): Float = max(0.0, min(2.0, v)).toFloat()

        private val VALID_HEADINGS = setOf("left", "center", "right")
        private val VALID_LOCATIONS = setOf("far left", "left", "center-left", "center", "center-right", "right", "far right")

        private fun coerceHeading(v: String) = v.trim().lowercase().let { s ->
            when {
                s in VALID_HEADINGS -> s
                s in setOf("l", "ccw", "anticlockwise") -> "left"
                s in setOf("r", "cw", "clockwise") -> "right"
                else -> "center"
            }
        }

        private fun coerceLoc(v: String): String? = v.trim().lowercase().takeIf { it in VALID_LOCATIONS }

        // Structured output schemas — mirror the Pydantic models in vlm_client.py.
        private val NAV_SCHEMA = JSONObject("""
            {
              "type": "object",
              "title": "NavigationDecision",
              "properties": {
                "target_visible": {"type": "boolean"},
                "target_location": {"anyOf": [{"type": "string"}, {"type": "null"}], "default": null},
                "heading": {"type": "string", "enum": ["left", "center", "right"]},
                "drive_distance_m": {"type": "number", "minimum": 0.0, "maximum": 2.0},
                "reason": {"type": "string"}
              },
              "required": ["target_visible", "heading", "drive_distance_m", "reason"]
            }""")

        private val VERIFY_SCHEMA = JSONObject("""
            {
              "type": "object",
              "title": "VerificationDecision",
              "properties": {
                "confirmed": {"type": "boolean"},
                "matches": {"type": "array", "items": {"type": "string"}, "default": []},
                "mismatches": {"type": "array", "items": {"type": "string"}, "default": []},
                "reason": {"type": "string", "default": ""}
              },
              "required": ["confirmed"]
            }""")
    }
}
