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
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * TeleOp - Blue
 *
 * CONTROLS:
 *   Left Stick: Field-centric drive
 *   Right Stick X: Turn
 *   Right Trigger: Intake
 *   Left Trigger: Reverse intake
 *   Cross (X): Gate open/close
 *   Dpad Up: Manual mid shot (disables auto-aim)
 *   Dpad Down: Manual far shot (disables auto-aim)
 *   Left Bumper: Toggle SOTM auto-aim
 *   Square: Reset pose to (136, 8, 180°)
 *   Triangle: Reset turret to goal
 *   Circle: Clear turret offset
 *   Dpad Left/Right: Nudge turret
 *
 * SOTM (Left Bumper):
 *   When ON: AutoAim controls turret, flywheel, and hood automatically.
 *   When OFF: Manual flywheel/hood via dpad, turret still locked to real goal.
 */
@TeleOp(name = "TeleOp - Blue", group = "Competition")
class TeleOpBlue : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry

    // Manual presets (used when AutoAim is OFF)
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
        Turret.alliance = Turret.Alliance.BLUE
        Drive.alliance = Drive.Alliance.BLUE
        Drive.loadSavedPose()
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()

        // Start with turret locked on real goal
        Turret.lock()
        bindControls()
    }

    private fun bindControls() {
        // ========== INTAKE ==========
        Gamepads.gamepad1.rightTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.run)
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
            Hood.mid()
        }

        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            AutoAim.disable()
            FlyWheel.setVelocity(farVelocity)
            Hood.far()
        }

        // ========== TURRET NUDGE ==========
        Gamepads.gamepad1.dpadLeft.whenBecomesTrue { Turret.nudgeLeft() }
        Gamepads.gamepad1.dpadRight.whenBecomesTrue { Turret.nudgeRight() }

        // ========== AUTO AIM / SOTM TOGGLE ==========
        Gamepads.gamepad1.leftBumper.whenBecomesTrue {
            AutoAim.toggle()
        }

        // ========== POSE RESET ==========
        Gamepads.gamepad1.square.whenBecomesTrue {
            follower.pose = Pose(136.0, 8.0, Math.toRadians(180.0))
            Turret.lock()
        }

        // ========== TURRET RESET ==========
        Gamepads.gamepad1.triangle.whenBecomesTrue {
            Turret.resetToGoal()
        }

        Gamepads.gamepad1.circle.whenBecomesTrue {
            Turret.clearOffset()
        }
    }

    override fun onUpdate() {
        // Drive.periodic() handles update(). AutoAim.periodic() handles SOTM.
        // Nothing needed here — subsystem periodic() calls do all the work.
        updateTelemetry()
    }

    override fun onStop() {
        AutoAim.disable()
        Intake.stop()
        Gate.close()
        Hood.close()
        FlyWheel.stop()
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