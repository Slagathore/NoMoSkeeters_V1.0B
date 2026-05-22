// Root build file. Plugin versions are declared here (apply false) and applied
// in the :app module. Toolchain: AGP 8.13.0 / Kotlin 2.0.20 / Gradle 8.13 / JDK 17.
// AGP 8.13 requires Gradle 8.13+ — the wrapper is set to match (a too-new AGP on
// a too-old Gradle is what produced the ":app:testApi ... locked upon creation"
// error before). If that error returns after this change it's almost certainly
// stale Gradle state from the version flip-flopping: File > Invalidate Caches /
// Restart and delete android/.gradle, then re-sync. Known-good fallback if it
// truly won't take: AGP 8.7.2 / Gradle 8.9.
plugins {
    id("com.android.application") version "8.13.0" apply false
    id("org.jetbrains.kotlin.android") version "2.0.20" apply false
}
