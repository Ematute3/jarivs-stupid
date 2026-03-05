package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import kotlin.math.*

/**
 * Global Constants for Revival Robot
 */

// ==================== ALLIANCE (SINGLE SOURCE OF TRUTH) ====================
/**
 * Shared alliance config. Set once in onInit(), read everywhere.
 * Eliminates the old pattern of setting Drive.alliance AND Turret.alliance separately.
 *
 * Usage:
 *   AllianceConfig.alliance = AllianceConfig.Alliance.RED  // set once
 *   AllianceConfig.goalX  // read from Turret, Drive, AutoAim, etc.
 */
object AllianceConfig {
    enum class Alliance { RED, BLUE }

    var alliance = Alliance.BLUE

    val goalX: Double
        get() = if (alliance == Alliance.RED) FieldConstants.RED_GOAL_X else FieldConstants.BLUE_GOAL_X

    val goalY: Double
        get() = FieldConstants.GOAL_Y
}

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

    const val DISTANCE_CLOSE_THRESHOLD = 0.862
    const val DISTANCE_MID_THRESHOLD = 2.587
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
    @JvmField var TOF_BASE = 0.3
    @JvmField var TOF_DISTANCE_SCALE = 0.2
    @JvmField var VELOCITY_THRESHOLD = 6.0
    @JvmField var VELOCITY_ALPHA = 0.3
    @JvmField var LEAD_GAIN = 1.0
}