plugins {
    id("com.android.application")
    kotlin("android")
}

android {
    namespace = "com.derpbot.app"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.derpbot.app"
        minSdk = 26          // BLE peripheral APIs + runtime perms baseline
        targetSdk = 34
        versionCode = 1
        versionName = "0.1"
    }

    buildFeatures {
        viewBinding = true
    }

    // Bundle the repo-root shared/ VLM brain (prompts + schema) as app assets so
    // the robot reads the SAME source of truth as the Python sim agent (#19).
    // At runtime: assets/prompts/detection_system.txt, assets/vlm_schema.json …
    sourceSets["main"].assets.srcDir("../../shared")

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }
}

dependencies {
    implementation(project(":rvr"))
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("androidx.activity:activity-ktx:1.9.2")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.8.1")
    implementation("com.google.android.material:material:1.12.0")
}
