package org.firstinspires.ftc.teamcode.Shooter.Hood

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.HoodConstants

/**
 * Hood Subsystem
 * Controls shooting angle based on distance to goal
 *
 * PRESETS:
 * - CLOSE: For shots < 1m (near goal)
 * - MID: For shots 1-2m
 * - FAR: For shots > 2m
 */
object Hood : Subsystem {
    private var hoodServo = ServoEx("hood")

    enum class HoodPreset {
        CLOSE,
        MID,
        FAR
    }

    var currentPreset = HoodPreset.CLOSE
    var currentPosition = HoodConstants.HOOD_CLOSE

    override fun initialize() {
        close()
    }

    // ==================== PRESETS ====================
    val close = InstantCommand {
        setPosition(HoodConstants.HOOD_CLOSE)
        currentPreset = HoodPreset.CLOSE
    }

    val mid = InstantCommand {
        setPosition(HoodConstants.HOOD_MID)
        currentPreset = HoodPreset.MID
    }

    val far = InstantCommand {
        setPosition(HoodConstants.HOOD_FAR)
        currentPreset = HoodPreset.FAR
    }

    // ==================== CONTROL ====================
    /**
     * Set exact servo position (0-1)
     */
    fun setPosition(position: Double) {
        val clamped = position.coerceIn(HoodConstants.servoMinPosition, HoodConstants.servoMaxPosition)
        currentPosition = clamped
        hoodServo.position = clamped
    }

    // ==================== AUTO AIM HELPER ====================
    /**
     * Get hood position for a given distance (called by AutoAim)
     * @param distanceMeters Distance to goal in meters
     * @return Servo position (0-1)
     */
    fun getPositionForDistance(distanceMeters: Double): Double {
        return when {
            distanceMeters < HoodConstants.DISTANCE_CLOSE_THRESHOLD -> HoodConstants.HOOD_CLOSE
            distanceMeters < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                val t = (distanceMeters - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                HoodConstants.HOOD_CLOSE + t * (HoodConstants.HOOD_MID - HoodConstants.HOOD_CLOSE)
            }
            else -> {
                val t = (distanceMeters - HoodConstants.DISTANCE_MID_THRESHOLD) / 2.0
                HoodConstants.HOOD_MID + t.coerceAtMost(1.0) * (HoodConstants.HOOD_FAR - HoodConstants.HOOD_MID)
            }
        }
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        ActiveOpMode.telemetry.addData("Hood/Position", "%.2f".format(currentPosition))
        ActiveOpMode.telemetry.addData("Hood/Preset", currentPreset.name)
    }
}