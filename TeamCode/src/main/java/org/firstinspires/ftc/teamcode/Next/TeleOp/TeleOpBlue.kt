package org.firstinspires.ftc.teamcode.Next.TeleOp

import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.AllianceConfig
import org.firstinspires.ftc.teamcode.GateConstants
import org.firstinspires.ftc.teamcode.HoodConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Next.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "TeleOp - Blue", group = "Competition")
class TeleOpBlue : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry

    private val midVelocity = 1500.0
    private val farVelocity = 1900.0

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Drive, Intake, Gate, FlyWheel, Turret, Hood, AutoAim
            ),
            BulkReadComponent, BindingsComponent
        )
    }

    override fun onInit() {
        // Single source of truth — set once, read everywhere
        AllianceConfig.alliance = AllianceConfig.Alliance.BLUE
        Drive.loadSavedPose()
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()

        Turret.lock()
        bindControls()
    }

    private fun bindControls() {
        // ========== INTAKE ==========
        Gamepads.gamepad1.rightTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.intake)
            .whenBecomesFalse(Intake.stop)

        Gamepads.gamepad1.leftTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)

        // ========== GATE ==========
        Gamepads.gamepad1.cross
            .whenBecomesTrue(Gate.open)
            .whenBecomesFalse(Gate.close)

        // ========== MANUAL SHOOTING (disables AutoAim) ==========
        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            AutoAim.disable()
            FlyWheel.setVelocity(midVelocity)
            Hood.setPosition(HoodConstants.HOOD_MID)
            Hood.currentPreset = Hood.HoodPreset.MID
        }

        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            AutoAim.disable()
            FlyWheel.setVelocity(farVelocity)
            Hood.setPosition(HoodConstants.HOOD_FAR)
            Hood.currentPreset = Hood.HoodPreset.FAR
        }

        // ========== TURRET NUDGE ==========
        Gamepads.gamepad2.dpadLeft.whenBecomesTrue { Turret.nudgeLeft() }
        Gamepads.gamepad2.dpadRight.whenBecomesTrue { Turret.nudgeRight() }

        // ========== AUTO AIM / SOTM TOGGLE ==========
        Gamepads.gamepad1.leftBumper.whenBecomesTrue {
            AutoAim.toggle()
        }

        // ========== POSE RESET ==========
        Gamepads.gamepad2.square.whenBecomesTrue {
            follower.pose = Pose(136.0, 8.0, Math.toRadians(180.0))
            Turret.lock()
        }

        // ========== TURRET RESET ==========
        Gamepads.gamepad2.triangle.whenBecomesTrue {
            Turret.resetToGoal()
        }

        Gamepads.gamepad2.circle.whenBecomesTrue {
            Turret.clearOffset()
        }
    }

    override fun onUpdate() {
        updateTelemetry()
    }

    override fun onStop() {
        // Call hardware directly — scheduled commands won't run after stop
        AutoAim.disable()
        Intake.run(0.0)
        Gate.setPosition(GateConstants.GATE_CLOSED)
        Hood.setPosition(HoodConstants.HOOD_CLOSE)
        FlyWheel.setVelocity(0.0)
        Turret.stop()
        Drive.savePose()
    }

    private fun updateTelemetry() {
        telemetry.addData("=== BLUE TELEOP ===", "")
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°".format(
            Drive.currentX, Drive.currentY, Math.toDegrees(Drive.currentHeading)))
        telemetry.addData("Speed", "%.1f in/s".format(Drive.speed))
        telemetry.addData("AutoAim", if (AutoAim.enabled) "ON" else "OFF")
        telemetry.addData("SOTM Active", if (AutoAim.isSOTMActive) "YES" else "NO")
        telemetry.update()
        panelsTelemetry.update()
    }
}