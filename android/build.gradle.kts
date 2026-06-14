// Root build file. Plugin versions are declared here with `apply false`;
// modules opt in via the `plugins {}` block in their own build.gradle.kts.
plugins {
    id("com.android.application") version "8.5.2" apply false
    kotlin("android") version "2.0.20" apply false
    kotlin("jvm") version "2.0.20" apply false
    kotlin("plugin.serialization") version "2.0.20" apply false
}
