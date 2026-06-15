package com.derpbot.app.vlm

data class VlmResult(
    val targetVisible: Boolean,
    val heading: String,           // "left" | "center" | "right"
    val driveDistanceM: Float,     // [0.0, 2.0]
    val targetLocation: String?,   // "far left" … "far right", or null
    val reason: String,
)

data class VerifyResult(
    val confirmed: Boolean,
    val matches: List<String> = emptyList(),
    val mismatches: List<String> = emptyList(),
    val reason: String = "",
)
