plugins {
    id("com.android.application")
}

android {
    namespace = "com.nomoskeeters.sensor"
    compileSdk = 37

    defaultConfig {
        applicationId = "com.nomoskeeters.sensor"
        // minSdk 31 (Android 12): MediaCodec KEY_LATENCY low-latency encoding
        // and PowerManager thermal listeners both land at API 31.
        minSdk = 31
        targetSdk = 37
        versionCode = 1
        versionName = "1.0"
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro",
            )
        }
    }

    buildFeatures {
        viewBinding = true
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}

dependencies {
    implementation("androidx.core:core-ktx:1.18.0")
    implementation("androidx.appcompat:appcompat:1.7.1")
    implementation("com.google.android.material:material:1.14.0")
    implementation("androidx.constraintlayout:constraintlayout:2.2.1")
    implementation("androidx.lifecycle:lifecycle-service:2.10.0")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.10.0")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.11.0")

    // org.json ships in the Android runtime, but the local unit tests run on a
    // plain JVM that doesn't have it — pull it in for tests so the pure-Kotlin
    // protocol/command logic is testable without an emulator.
    testImplementation("junit:junit:4.13.2")
    testImplementation("org.json:json:20251224")
    testImplementation("org.jetbrains.kotlinx:kotlinx-coroutines-test:1.11.0")
}
