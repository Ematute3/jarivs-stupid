package org.firstinspires.ftc.teamcode.Next.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.AllianceConfig
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.TurretConstants.motorGearTeeth
import org.firstinspires.ftc.teamcode.TurretConstants.outputGearTeeth
import kotlin.math.*

const val TURRET_MIN_ANGLE = -135.0
const val TURRET_MAX_ANGLE = 90.0
const val ENCODER_CPR = 4000

@Configurable
object Turret : Subsystem {

    enum class State {
        IDLE,
        MANUAL,
        LOCKED,
        LOCKED_EXTERNAL,
        RESET,
        RESET_TO_GOAL
    }

    // ==================== TUNABLE ====================
    @JvmField var kP: Double = 0.5
    @JvmField var kI: Double = 0.0
    @JvmField var kD: Double = 0.001

    @JvmField var ffKV: Double = 0.11
    @JvmField var ffKA: Double = 0.0
    @JvmField var ffKS: Double = 0.05

    @JvmField var maxPower: Double = 1.0
    @JvmField var alignmentTolerance: Double = 2.0
    @JvmField var nudgeStepDegrees: Double = 1.0

    // ==================== HARDWARE ====================
    var motor = MotorEx("turret")

    // ==================== STATE ====================
    var currentState = State.IDLE
    var manualPower = 0.0
    var targetYaw = 0.0
    var isLocked = false
    var angleOffsetRad: Double = 0.0

    // External target angle (set by AutoAim for SOTM)
    private var externalFieldAngle = 0.0

    // ==================== VELOCITY TRACKING ====================
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    var robotAngularVelocity = 0.0

    // ==================== GOAL (from shared AllianceConfig) ====================
    private val goalX: Double get() = AllianceConfig.goalX
    private val goalY: Double get() = AllianceConfig.goalY

    private val MIN_ANGLE = Math.toRadians(TURRET_MIN_ANGLE)
    private val MAX_ANGLE = Math.toRadians(TURRET_MAX_ANGLE)

    // ==================== CONTROLLER ====================
    private var controller = buildController()
    private var lastKP = kP; private var lastKI = kI; private var lastKD = kD
    private var lastFFKV = ffKV; private var lastFFKA = ffKA; private var lastFFKS = ffKS

    private fun buildController() = controlSystem {
        posPid(kP, kI, kD)
        basicFF(ffKV, ffKA, ffKS)
    }

    private fun rebuildControllerIfNeeded() {
        if (kP != lastKP || kI != lastKI || kD != lastKD ||
            ffKV != lastFFKV || ffKA != lastFFKA || ffKS != lastFFKS
        ) {
            controller = buildController()
            lastKP = kP; lastKI = kI; lastKD = kD
            lastFFKV = ffKV; lastFFKA = ffKA; lastFFKS = ffKS
        }
    }

    // ==================== INIT ====================

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

    // ==================== PERIODIC ====================

    override fun periodic() {
        rebuildControllerIfNeeded()

        if (currentState == State.LOCKED || currentState == State.LOCKED_EXTERNAL ||
            currentState == State.RESET_TO_GOAL
        ) {
            updateRobotAngularVelocity()
        }

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-1.0, 1.0)
            State.LOCKED -> runLockedControl()
            State.LOCKED_EXTERNAL -> runLockedExternalControl()
            State.RESET -> runResetControl()
            State.RESET_TO_GOAL -> runResetToGoal()
        }

        telemetry.addData("Turret/State", currentState.name)
        telemetry.addData("Turret/Yaw", "%.2f°".format(Math.toDegrees(getYaw())))
        telemetry.addData("Turret/Target", "%.2f°".format(Math.toDegrees(targetYaw)))
        telemetry.addData("Turret/Locked", if (isLocked) "YES" else "NO")
        telemetry.addData("Turret/Power", motor.power)
    }

    // ==================== CONTROL ====================

    private fun applyControl(targetYaw: Double, targetVelocity: Double = 0.0) {
        val clampedTarget = targetYaw.coerceIn(MIN_ANGLE, MAX_ANGLE)
        val currentYaw = getYaw()

        controller.goal = KineticState(clampedTarget, targetVelocity)
        val power = controller.calculate(KineticState(currentYaw, 0.0))

        motor.power = power.coerceIn(-maxPower, maxPower)
        this.targetYaw = clampedTarget
    }

    /** LOCKED — aims at real goal (stationary shooting) */
    private fun runLockedControl() {
        val fieldAngle = atan2(goalY - Drive.currentY, goalX - Drive.currentX)
        val rawTarget = normalizeAngle(fieldAngle - Drive.currentHeading + angleOffsetRad)
        applyControl(rawTarget, -robotAngularVelocity)
    }

    /** LOCKED_EXTERNAL — aims at angle provided by AutoAim (SOTM) */
    private fun runLockedExternalControl() {
        val rawTarget = normalizeAngle(externalFieldAngle - Drive.currentHeading + angleOffsetRad)
        applyControl(rawTarget, -robotAngularVelocity)
    }

    private fun runResetControl() {
        val currentYaw = getYaw()
        val error = normalizeAngle(0.0 - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        if (errorDeg < alignmentTolerance) {
            motor.power = 0.0
            targetYaw = 0.0
            currentState = State.IDLE
            return
        }
        applyControl(0.0)
    }

    private fun runResetToGoal() {
        val fieldAngle = atan2(goalY - Drive.currentY, goalX - Drive.currentX)
        val targetAngle = normalizeAngle(fieldAngle - Drive.currentHeading + angleOffsetRad)

        val currentYaw = getYaw()
        val error = normalizeAngle(targetAngle - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        if (errorDeg < alignmentTolerance) {
            motor.power = 0.0
            targetYaw = targetAngle
            currentState = State.LOCKED
            isLocked = true
            return
        }
        applyControl(targetAngle, 0.0)
    }

    // ==================== ANGULAR VELOCITY ====================

    private fun updateRobotAngularVelocity() {
        if (!Drive.poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt < 0.01 || dt > 0.2) {
            if (dt > 0.2) robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        val currentHeading = Drive.currentHeading
        if (currentHeading.isNaN() || currentHeading.isInfinite()) {
            robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        val deltaHeading = normalizeAngle(currentHeading - lastRobotHeading)
        robotAngularVelocity = deltaHeading / dt

        lastRobotHeading = currentHeading
        velTimer.reset()
    }

    // ==================== PUBLIC API ====================

    /** Lock turret to a field-relative angle (called by AutoAim for SOTM) */
    fun lockToTarget(fieldAngleRad: Double) {
        externalFieldAngle = fieldAngleRad
        if (currentState != State.LOCKED_EXTERNAL) {
            lastRobotHeading = Drive.currentHeading
            velTimer.reset()
            isLocked = true
            currentState = State.LOCKED_EXTERNAL
        }
    }

    /** Lock turret — auto-aim at real goal (stationary shooting) */
    fun lock() {
        lastRobotHeading = Drive.currentHeading
        velTimer.reset()
        isLocked = true
        currentState = State.LOCKED
    }

    fun resetToGoal() {
        lastRobotHeading = Drive.currentHeading
        velTimer.reset()
        isLocked = false
        currentState = State.RESET_TO_GOAL
    }

    fun stop() {
        currentState = State.IDLE
        isLocked = false
        motor.power = 0.0
    }

    fun setManual(power: Double) {
        currentState = State.MANUAL
        isLocked = false
        manualPower = power
    }

    fun resetToCenter() {
        currentState = State.RESET
        isLocked = false
    }

    fun nudgeLeft() {
        angleOffsetRad += Math.toRadians(nudgeStepDegrees)
    }

    fun nudgeRight() {
        angleOffsetRad -= Math.toRadians(nudgeStepDegrees)
    }

    fun clearOffset() {
        angleOffsetRad = 0.0
    }

    // ==================== UTILITIES ====================

    fun getYawDegrees(): Double = Math.toDegrees(getYaw())

    fun getYaw(): Double = normalizeAngle(
        motor.currentPosition.toDouble() * ticksToRadians()
    )

    private fun ticksToRadians(): Double {
        val gearRatio = outputGearTeeth.toDouble() / motorGearTeeth.toDouble()
        return (2.0 * PI / ENCODER_CPR) * (1.0 / gearRatio)
    }

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }
}