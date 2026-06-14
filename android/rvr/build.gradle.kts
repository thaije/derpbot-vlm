// Pure Kotlin/JVM module: the RVR wire protocol. No Android dependencies, so
// its unit tests run on a plain JVM (./gradlew :rvr:test) without an emulator
// or the Android SDK — this is the byte-level-verifiable core of Step 1 (#19).
plugins {
    kotlin("jvm")
}

dependencies {
    testImplementation(kotlin("test"))
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(17)
}
