// Root build file. Plugin versions are declared here (apply false) and applied
// in the :app module. Toolchain: AGP 8.7.2 / Kotlin 2.0.20 / Gradle 8.9 / JDK 17.
// This is a deliberately conservative, battle-tested pairing. Gradle 8.11+
// locks configuration "roles" strictly enough that some AGP/plugin combos fail
// configuring :app:testApi ("Cannot change the allowed usage..."); AGP 8.7.2 on
// Gradle 8.9 sidesteps that. Android Studio may offer to upgrade — only accept
// if you also bump Gradle to a version it has actually validated together.
plugins {
    id("com.android.application") version "8.7.2" apply false
    id("org.jetbrains.kotlin.android") version "2.0.20" apply false
}
