package org.firstinspires.ftc.teamcode.Lower.Intake

import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.IntakeConstants

/**
 * Intake Subsystem
 * Controls roller intake for picking up game pieces
 */
object Intake : Subsystem {
    var intakeMotor = MotorEx("intake")

    enum class IntakeState {
        STOPPED,
        INTAKING,
        REVERSING
    }

    var intakeState = IntakeState.STOPPED

    override fun initialize() {
        intakeMotor.motor.direction = DcMotorSimple.Direction.FORWARD
        stop()
    }

    internal fun run(iPow: Double) {
        intakeMotor.power = iPow
    }

    val run = InstantCommand {
        intakeMotor.power = IntakeConstants.INTAKE_POWER
        intakeState = IntakeState.INTAKING
    }

    val reverse = InstantCommand {
        intakeMotor.power = IntakeConstants.REVERSE_POWER
        intakeState = IntakeState.REVERSING
    }

    val stop = InstantCommand {
        intakeMotor.power = 0.0
        intakeState = IntakeState.STOPPED
    }

    override fun periodic() {
        ActiveOpMode.telemetry.addData("Intake/State", intakeState.name)
    }
}