// Root build file. Plugin versions are declared here (apply false) and applied
// in the :app module. Toolchain: AGP 8.13.2 / Kotlin 2.0.20 → needs Gradle 8.13+
// and JDK 17 (Android Studio bundles a suitable JDK).
plugins {
    id("com.android.application") version "8.13.2" apply false
    id("org.jetbrains.kotlin.android") version "2.0.20" apply false
}
