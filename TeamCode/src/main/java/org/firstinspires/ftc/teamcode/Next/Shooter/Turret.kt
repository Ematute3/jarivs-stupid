package org.firstinspires.ftc.teamcode.Next.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.FieldConstants.BLUE_GOAL_X
import org.firstinspires.ftc.teamcode.FieldConstants.GOAL_Y
import org.firstinspires.ftc.teamcode.FieldConstants.RED_GOAL_X
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.TurretConstants.motorGearTeeth
import org.firstinspires.ftc.teamcode.TurretConstants.outputGearTeeth

import kotlin.math.*

/**
 * Turret Subsystem
 * Controls the shooting turret with auto-aim
 * 
 * KEY FEATURES:
 * 1. Auto-lock - automatically aims at goal based on robot position
 * 2. Robot rotation compensation - adjusts for robot spinning
 * 3. Angle limits - clamps to physical limits (-135 to 90 degrees)
 * 4. Reset to goal - recenters turret to point at goal
 * 5. Nudge - small adjustments for fine-tuning aim
 * 6. Live tuning - PID values can be adjusted via Panels
 * 
 * STATES:
 * - IDLE: Motor off
 * - MANUAL: Driver controls directly
 * - LOCKED: Auto-aiming at goal
 * - RESET: Moving to center (0 degrees)
 * - RESET_TO_GOAL: Moving to point at goal
 */

// Physical angle limits (degrees)
// These prevent turret from hitting wires/limits
const val TURRET_MIN_ANGLE = -135.0
const val TURRET_MAX_ANGLE = 90.0

// Encoder counts per revolution (East Loop encoder)
// 4000 CPR quadrature encoder
const val ENCODER_CPR = 4000

@Configurable
object Turret : Subsystem {

    // ==================== STATES ====================
    enum class State { 
        IDLE,           // Motor off, no control
        MANUAL,         // Driver has direct control
        LOCKED,         // Auto-aiming at goal
        RESET,          // Moving to center (0 degrees)
        RESET_TO_GOAL   // Moving to point at goal
    }

    // ==================== TUNABLE VALUES ====================
    // These can be adjusted live via Panels
    
    @JvmField var kP: Double = 0.5      // Proportional gain - response speed
    @JvmField var kI: Double = 0.0      // Integral gain - steady-state error
    @JvmField var kD: Double = 0.001    // Derivative gain - dampening
    
    @JvmField var ffKV: Double = 0.11   // Feedforward velocity - hold position
    @JvmField var ffKA: Double = 0.0     // Feedforward acceleration
    @JvmField var ffKS: Double = 0.05    // Feedforward static friction
    
    @JvmField var maxPower: Double = 1.0      // Max motor power
    @JvmField var alignmentTolerance: Double = 2.0  // Degrees - "close enough"
    @JvmField var sotmLookahead: Double = 0.0     // SOTM time lookahead
    @JvmField var nudgeStepDegrees: Double = 1.0   // Degrees per nudge press

    // ==================== HARDWARE ====================
    var motor = MotorEx("turret")

    // ==================== STATE VARIABLES ====================
    var currentState = State.IDLE
    var manualPower = 0.0
    var targetYaw = 0.0  // Where we want turret to point
    var isLocked = false  // Whether we're on target
    
    // Angle offset in radians - for nudging
    var angleOffsetRad: Double = 0.0

    // ==================== VELOCITY TRACKING ====================
    // Track how fast robot is spinning
    // This is used to compensate for robot rotation
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    var robotAngularVelocity = 0.0  // Radians per second

    // ==================== ALLIANCE ====================
    enum class Alliance { RED, BLUE }
    var alliance = Alliance.BLUE

    // Goal position based on alliance
    val goalX: Double get() = if (alliance == Alliance.RED) RED_GOAL_X else BLUE_GOAL_X
    val goalY = GOAL_Y

    // Convert limits to radians
    private val MIN_ANGLE = Math.toRadians(TURRET_MIN_ANGLE)
    private val MAX_ANGLE = Math.toRadians(TURRET_MAX_ANGLE)

    // ==================== CONTROLLER ====================
    // PID + Feedforward controller
    // Rebuilds each cycle for live tuning
    private var controller = buildController()

    /**
     * Build PID/FF controller
     * Reads from @JvmField values so changes in Panels take effect
     */
    private fun buildController() = controlSystem {
        posPid(kP, kI, kD)
        basicFF(ffKV, ffKA, ffKS)
    }

    /**
     * Initialize turret
     * Resets encoder, sets up controller
     */
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        
        velTimer.reset()
        lastRobotHeading = Drive.currentHeading
        robotAngularVelocity = 0.0
        
        controller = buildController()
        targetYaw = getYaw()
        angleOffsetRad = 0.0
    }

    /**
     * Main loop - runs every cycle
     * Handles state machine and updates
     */
    override fun periodic() {
        // Rebuild controller each cycle for live tuning
        // This makes PID changes in Panels take effect immediately
        controller = buildController()
        
        // Update robot velocity tracking when locked or resetting
        if (currentState == State.LOCKED || currentState == State.RESET_TO_GOAL) {
            updateRobotAngularVelocity()
        }

        // State machine
        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-1.0, 1.0)
            State.LOCKED -> runLockedControl()
            State.RESET -> runResetControl()
            State.RESET_TO_GOAL -> runResetToGoal()
        }

        // Telemetry for debugging
        telemetry.addData("Turret/State", currentState.name)
        telemetry.addData("Turret/Yaw", "%.2f°".format(Math.toDegrees(getYaw())))
        telemetry.addData("Turret/Target", "%.2f°".format(Math.toDegrees(targetYaw)))
        telemetry.addData("Turret/Locked", if (isLocked) "YES" else "NO")
        telemetry.addData("Turret/Offset", "%.2f°".format(Math.toDegrees(angleOffsetRad)))
        telemetry.addData("Turret/RobotVel", "%.2f°/s".format(Math.toDegrees(robotAngularVelocity)))
        telemetry.addData("Turret/SOTM", "lookahead=%.2fs".format(sotmLookahead))
        telemetry.addData("Turret/Power", motor.power)
        telemetry.update()
    }

    /**
     * Apply PID control to reach target angle
     * 
     * @param targetYaw Where we want turret to point (radians)
     * @param targetVelocity Velocity compensation (radians/sec)
     */
    private fun applyControl(targetYaw: Double, targetVelocity: Double = 0.0) {
        // Clamp to physical limits
        val clampedTarget = targetYaw.coerceIn(MIN_ANGLE, MAX_ANGLE)
        
        // Get current position
        val currentYaw = getYaw()
        
        // Set PID goal
        controller.goal = KineticState(clampedTarget, targetVelocity)
        
        // Calculate PID output
        // Uses current velocity for derivative term
        val power = controller.calculate(KineticState(currentYaw, 0.0))
        
        // Apply to motor
        motor.power = power.coerceIn(-maxPower, maxPower)
        
        // Store target for telemetry
        this.targetYaw = clampedTarget
    }

    /**
     * LOCKED state - Auto-aim at goal
     * Calculates angle to goal and adjusts for robot rotation
     */
    fun runLockedControl() {
        // Calculate angle from robot to goal (field-relative)
        val deltaX = goalX - Drive.currentX
        val deltaY = goalY - Drive.currentY
        val fieldAngleToGoal = atan2(deltaY, deltaX)

        // Convert to robot-relative angle
        val robotHeadingRad = Drive.currentHeading
        
        // Calculate target angle with offset (for nudging)
        // This is where turret should point to hit goal
        val rawTarget = normalizeAngle(fieldAngleToGoal - robotHeadingRad + angleOffsetRad)

        // Apply control with rotation compensation
        // If robot spins CW, turret must spin CCW to maintain aim
        applyControl(rawTarget, -robotAngularVelocity)
    }

    /**
     * RESET state - Move to center (0 degrees)
     */
    fun runResetControl() {
        val currentYaw = getYaw()
        val error = normalizeAngle(0.0 - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        // If close enough, stop
        if (errorDeg < alignmentTolerance) {
            motor.power = 0.0
            targetYaw = 0.0
            currentState = State.IDLE
            return
        }

        // Move to center
        applyControl(0.0)
    }

    /**
     * RESET_TO_GOAL state - Point turret at goal from current position
     * 
     * Use this when turret is misaligned and you want to re-center on goal
     * Press triangle in teleop to activate
     */
    fun runResetToGoal() {
        // Calculate angle to goal
        val deltaX = goalX - Drive.currentX
        val deltaY = goalY - Drive.currentY
        val fieldAngleToGoal = atan2(deltaY, deltaX)
        
        // Convert to robot-relative
        val robotHeadingRad = Drive.currentHeading
        val targetAngle = normalizeAngle(fieldAngleToGoal - robotHeadingRad + angleOffsetRad)
        
        // Get current position
        val currentYaw = getYaw()
        val error = normalizeAngle(targetAngle - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        // If close enough to target, lock on
        if (errorDeg < alignmentTolerance) {
            motor.power = 0.0
            targetYaw = targetAngle
            currentState = State.LOCKED  // Transition to locked
            isLocked = true
            return
        }

        // Move toward target
        applyControl(targetAngle, 0.0)
    }

    /**
     * Update robot angular velocity
     * Tracks how fast robot is spinning
     * Used for rotation compensation in auto-aim
     */
    private fun updateRobotAngularVelocity() {
        // Can't track without valid pose
        if (!Drive.poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        
        // Check for reasonable time delta
        if (dt < 0.01 || dt > 0.2) {
            if (dt > 0.2) robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        val currentHeading = Drive.currentHeading
        
        // Validate heading
        if (currentHeading.isNaN() || currentHeading.isInfinite()) {
            robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        // Calculate angular velocity = change in angle / time
        val deltaHeading = normalizeAngle(currentHeading - lastRobotHeading)
        robotAngularVelocity = deltaHeading / dt
        
        // Store for next cycle
        lastRobotHeading = currentHeading
        velTimer.reset()
    }

    // ==================== PUBLIC METHODS ====================

    /**
     * Nudge turret left
     * Adds to angle offset
     */
    fun nudgeLeft() {
        angleOffsetRad += Math.toRadians(nudgeStepDegrees)
    }

    /**
     * Nudge turret right
     * Subtracts from angle offset
     */
    fun nudgeRight() {
        angleOffsetRad -= Math.toRadians(nudgeStepDegrees)
    }

    /**
     * Clear angle offset
     * Resets nudging
     */
    fun clearOffset() {
        angleOffsetRad = 0.0
    }

    /**
     * Lock turret - Enable auto-aim
     * This is the normal shooting mode
     */
    fun lock() {
        lastRobotHeading = Drive.currentHeading
        velTimer.reset()
        isLocked = true
        currentState = State.LOCKED
    }

    /**
     * Reset to goal - Point turret at goal from current position
     * Use this when turret is misaligned
     * Press triangle in teleop
     */
    fun resetToGoal() {
        lastRobotHeading = Drive.currentHeading
        velTimer.reset()
        isLocked = false
        currentState = State.RESET_TO_GOAL
    }

    /**
     * Stop turret
     */
    fun stop() {
        currentState = State.IDLE
        isLocked = false
        motor.power = 0.0
    }

    /**
     * Manual control mode
     * Driver has direct control
     */
    fun setManual(power: Double) {
        currentState = State.MANUAL
        isLocked = false
        manualPower = power
    }

    /**
     * Reset to center
     */
    fun resetToCenter() {
        currentState = State.RESET
        isLocked = false
    }

    /**
     * Get current yaw in degrees
     */
    fun getYawDegrees(): Double = Math.toDegrees(getYaw())

    /**
     * Get current yaw in radians
     * Reads from encoder
     */
    fun getYaw(): Double = normalizeAngle(
        motor.currentPosition.toDouble() * ticksToRadians()
    )

    /**
     * Convert encoder ticks to radians
     * Accounts for gear ratio
     */
    private fun ticksToRadians(): Double {
        val gearRatio = outputGearTeeth.toDouble() / motorGearTeeth.toDouble()
        // ticks * (2π radians / CPR) / gearRatio
        return (2.0 * PI / ENCODER_CPR) * (1.0 / gearRatio)
    }

    /**
     * Normalize angle to -π to π range
     * Prevents angle wraparound issues
     */
    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }
}
