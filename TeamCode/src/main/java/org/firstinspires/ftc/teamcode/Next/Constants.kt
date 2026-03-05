package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import kotlin.math.*

/**
 * Global Constants for Revival Robot
 */

// ==================== FIELD CONSTANTS ====================
@Configurable
object FieldConstants {

    const val GOAL_Y = 144.0
    const val RED_GOAL_X = 144.0
    const val BLUE_GOAL_X = 0.0

    fun isInMainShootingZone(x: Double, y: Double): Boolean {
        val x1 = 0.0; val y1 = 144.0
        val x2 = 144.0; val y2 = 144.0
        val x3 = 72.0; val y3 = 72.0
        return barycentric(x, y, x1, y1, x2, y2, x3, y3) >= 0
    }

    fun isInSecondaryShootingZone(x: Double, y: Double): Boolean {
        val x1 = 96.0; val y1 = 0.0
        val x2 = 48.0; val y2 = 0.0
        val x3 = 72.0; val y3 = 24.0
        return barycentric(x, y, x1, y1, x2, y2, x3, y3) >= 0
    }

    private fun barycentric(
        px: Double, py: Double,
        x1: Double, y1: Double,
        x2: Double, y2: Double,
        x3: Double, y3: Double
    ): Double {
        val denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        if (denom == 0.0) return -1.0
        val a = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / denom
        val b = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / denom
        val c = 1 - a - b
        return minOf(a, b, c)
    }
}

// ==================== SHOOTER CONSTANTS ====================
@Configurable
object ShooterConstants {
    /** Flywheel velocity presets (ticks/s) */
    const val FLYWHEEL_CLOSE_RPM = 1000.0
    const val FLYWHEEL_MID_RPM = 1500.0
    const val FLYWHEEL_FAR_RPM = 1900.0
}

// ==================== TURRET CONSTANTS ====================
@Configurable
object TurretConstants {
    @JvmField var motorGearTeeth = 29
    @JvmField var outputGearTeeth = 105
}

// ==================== HOOD CONSTANTS ====================
@Configurable
object HoodConstants {
    const val HOOD_CLOSE = 0.0
    const val HOOD_MID = 0.5
    const val HOOD_FAR = 0.7

    @JvmField var servoMinPosition = 0.0
    @JvmField var servoMaxPosition = 0.7

    const val DISTANCE_CLOSE_THRESHOLD = 0.862  // meters
    const val DISTANCE_MID_THRESHOLD = 2.587    // meters
}

// ==================== INTAKE CONSTANTS ====================
@Configurable
object IntakeConstants {
    const val INTAKE_POWER = 1.0
    const val REVERSE_POWER = -0.5
}

// ==================== GATE CONSTANTS ====================
object GateConstants {
    const val GATE_OPEN = 0.0
    const val GATE_CLOSED = 1.0
}

// ==================== SHOOTING ON THE MOVE ====================
@Configurable
object SOTMConstants {
    /**
     * SOTM uses a virtual goal to compensate for robot motion during ball flight.
     *
     * HOW IT WORKS:
     * 1. Estimate how long the ball takes to reach the goal (time of flight)
     * 2. During that time, robot motion causes the ball to drift
     * 3. Aim at a "virtual goal" offset to cancel that drift
     *
     * TIME OF FLIGHT MODEL:
     *   tof = TOF_BASE + TOF_DISTANCE_SCALE * distanceMeters
     *
     *   - TOF_BASE: minimum flight time at point-blank range
     *   - TOF_DISTANCE_SCALE: additional time per meter of distance
     *
     * VELOCITY COMPENSATION:
     *   virtualGoal = realGoal - robotVelocity * tof
     *   The subtraction cancels the ball's inherited drift from robot motion.
     *
     * TUNING PROCEDURE:
     *   See SOTM_TUNING_GUIDE.md for step-by-step instructions.
     */

    // Time of flight model: tof = base + scale * distance
    // Start conservative, then reduce base until shots land on time
    @JvmField var TOF_BASE = 0.3           // seconds — minimum flight time (tune first)
    @JvmField var TOF_DISTANCE_SCALE = 0.2 // seconds per meter — how much longer for far shots

    // Below this speed (inches/sec), use stationary aiming (no virtual goal)
    // Prevents velocity noise from causing turret jitter when standing still
    @JvmField var VELOCITY_THRESHOLD = 6.0

    // Velocity filter alpha (0-1). Higher = more responsive but noisier.
    // Lower = smoother but more lag. Start at 0.3.
    @JvmField var VELOCITY_ALPHA = 0.3

    // Lead compensation multiplier. 1.0 = physics-perfect lead.
    // Increase above 1.0 if shots consistently trail behind the target.
    // Decrease below 1.0 if shots consistently overshoot ahead.
    @JvmField var LEAD_GAIN = 1.0
}