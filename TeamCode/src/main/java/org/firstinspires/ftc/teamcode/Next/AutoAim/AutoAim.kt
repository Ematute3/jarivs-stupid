package org.firstinspires.ftc.teamcode.Next.AutoAim

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.HoodConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.SOTMConstants
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.ShooterConstants
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret

/**
 * Unified AutoAim / SOTM Pipeline
 *
 * This is the SINGLE SOURCE OF TRUTH for all aiming during shooting-on-the-move.
 * When enabled, it controls all three systems from one virtual goal:
 *
 *   1. TURRET — aims at virtual goal (compensates for robot translation)
 *   2. FLYWHEEL — continuous interpolation based on distance to virtual goal
 *   3. HOOD — continuous interpolation based on distance to virtual goal
 *
 * WHEN DISABLED:
 *   - Turret reverts to aiming at the real goal (stationary LOCKED mode)
 *   - Flywheel and hood are under manual control (dpad presets)
 *
 * VIRTUAL GOAL MATH:
 *   When robot moves, the ball inherits that velocity at launch.
 *   To cancel the drift:
 *     virtualGoal = realGoal - robotVelocity * timeOfFlight * leadGain
 *   The turret aims at virtualGoal. Flywheel/hood use distance to virtualGoal.
 *
 * STATIONARY FALLBACK:
 *   When robot speed < VELOCITY_THRESHOLD, virtual goal = real goal.
 *   This prevents noise-induced jitter during still shots.
 */
@Configurable
object AutoAim : Subsystem {

    // ==================== STATE ====================
    var enabled = false
        private set

    // Computed values (readable for telemetry / debugging)
    var currentDistanceMeters = 0.0
        private set
    var currentTOF = 0.0
        private set
    var targetVelocity = 0.0
        private set
    var targetHoodPosition = 0.0
        private set
    var targetFieldAngle = 0.0
        private set
    var isSOTMActive = false
        private set

    // ==================== INIT ====================

    override fun initialize() {
        enabled = false
        currentDistanceMeters = 0.0
        currentTOF = 0.0
        targetVelocity = 0.0
        targetHoodPosition = 0.0
        targetFieldAngle = 0.0
        isSOTMActive = false
    }

    // ==================== MAIN LOOP ====================

    override fun periodic() {
        if (!enabled) return

        // --- Step 1: Determine if we're moving fast enough for SOTM ---
        isSOTMActive = Drive.speed > SOTMConstants.VELOCITY_THRESHOLD

        // --- Step 2: Compute distance and time of flight ---
        // Use real goal distance for TOF estimate (close enough, avoids circular dependency)
        val realDistMeters = Drive.distanceToGoalMeters()
        currentTOF = Drive.estimateTimeOfFlight(realDistMeters)

        // --- Step 3: Compute virtual goal (or use real goal if stationary) ---
        val aimDistMeters: Double
        val aimFieldAngle: Double

        if (isSOTMActive) {
            // SOTM: aim at virtual goal to cancel ball drift
            aimDistMeters = Drive.distanceToVirtualGoal(currentTOF) * 0.0254
            aimFieldAngle = Drive.angleToVirtualGoalRad(currentTOF)
        } else {
            // Stationary: aim at real goal
            aimDistMeters = realDistMeters
            aimFieldAngle = Drive.angleToGoalRad()
        }

        currentDistanceMeters = aimDistMeters
        targetFieldAngle = aimFieldAngle

        // --- Step 4: Set turret to aim at computed angle ---
        // lockToTarget gives the turret a field-relative angle.
        // The turret internally converts to robot-relative and applies PID + rotation comp.
        Turret.lockToTarget(aimFieldAngle)

        // --- Step 5: Set flywheel velocity (continuous interpolation) ---
        targetVelocity = interpolateFlywheelVelocity(aimDistMeters)
        FlyWheel.setVelocity(targetVelocity)

        // --- Step 6: Set hood position (continuous interpolation) ---
        targetHoodPosition = Hood.getPositionForDistance(aimDistMeters)
        Hood.setPosition(targetHoodPosition)

        // --- Telemetry ---
        ActiveOpMode.telemetry.addData("AutoAim/Enabled", "ON")
        ActiveOpMode.telemetry.addData("AutoAim/SOTM", if (isSOTMActive) "ACTIVE (%.1f in/s)".format(Drive.speed) else "OFF")
        ActiveOpMode.telemetry.addData("AutoAim/Distance", "%.2f m".format(currentDistanceMeters))
        ActiveOpMode.telemetry.addData("AutoAim/TOF", "%.3f s".format(currentTOF))
        ActiveOpMode.telemetry.addData("AutoAim/Flywheel", "%.0f t/s".format(targetVelocity))
        ActiveOpMode.telemetry.addData("AutoAim/Hood", "%.3f".format(targetHoodPosition))
        ActiveOpMode.telemetry.addData("AutoAim/FieldAngle", "%.1f°".format(Math.toDegrees(targetFieldAngle)))
    }

    // ==================== FLYWHEEL INTERPOLATION ====================

    /**
     * Continuously interpolate flywheel velocity based on distance.
     * No more hard zone boundaries — smooth transitions.
     *
     * Close range:  constant FLYWHEEL_CLOSE_RPM
     * Mid range:    linear ramp from CLOSE to MID
     * Far range:    linear ramp from MID to FAR (clamped)
     */
    private fun interpolateFlywheelVelocity(distanceMeters: Double): Double {
        return when {
            distanceMeters < HoodConstants.DISTANCE_CLOSE_THRESHOLD ->
                ShooterConstants.FLYWHEEL_CLOSE_RPM

            distanceMeters < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                val t = (distanceMeters - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                ShooterConstants.FLYWHEEL_CLOSE_RPM + t *
                        (ShooterConstants.FLYWHEEL_MID_RPM - ShooterConstants.FLYWHEEL_CLOSE_RPM)
            }

            else -> {
                // Ramp from mid to far over an additional range equal to close->mid distance
                val range = HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD
                val t = ((distanceMeters - HoodConstants.DISTANCE_MID_THRESHOLD) / range).coerceIn(0.0, 1.0)
                ShooterConstants.FLYWHEEL_MID_RPM + t *
                        (ShooterConstants.FLYWHEEL_FAR_RPM - ShooterConstants.FLYWHEEL_MID_RPM)
            }
        }
    }

    // ==================== PUBLIC API ====================

    /**
     * Enable AutoAim / SOTM pipeline.
     * Takes over turret, flywheel, and hood.
     */
    fun enable() {
        enabled = true
    }

    /**
     * Disable AutoAim / SOTM pipeline.
     * Returns turret to stationary lock on real goal.
     * Flywheel and hood return to manual control.
     */
    fun disable() {
        enabled = false
        isSOTMActive = false
        // Return turret to normal locked mode (real goal)
        Turret.lock()
    }

    /**
     * Toggle on/off
     */
    fun toggle() {
        if (enabled) disable() else enable()
    }
}