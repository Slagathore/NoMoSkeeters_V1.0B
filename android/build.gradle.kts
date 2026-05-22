// Root build file. Toolchain: AGP 9.2.1 / Gradle 9.4.1 / JDK 17, settled by the
// Android Studio AGP Upgrade Assistant and verified building + running on-device
// (2026). AGP 9 has built-in Kotlin (android.builtInKotlin=true in
// gradle.properties), so there's no separate Kotlin plugin. Keep AGP and Gradle
// a matched pair — mismatching them is what produced the earlier
// ":app:testApi ... locked upon creation" configuration error.
plugins {
    id("com.android.application") version "9.2.1" apply false
}
