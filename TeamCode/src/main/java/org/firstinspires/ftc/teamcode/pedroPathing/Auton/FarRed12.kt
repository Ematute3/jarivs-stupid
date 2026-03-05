package org.firstinspires.ftc.teamcode.Autonomoous.Blue

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
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
import org.firstinspires.ftc.teamcode.Systems.ShooterCommands
import org.firstinspires.ftc.teamcode.Systems.initCommands
import org.firstinspires.ftc.teamcode.Systems.intakeAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Configurable
@Autonomous(name = "Red-Far", group = "Competition", preselectTeleOp = "TeleOp - Blue")
class AutoBlueMirrored : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Drive, Intake, Gate, FlyWheel, Turret, Hood
            ),
            BulkReadComponent, BindingsComponent
        )
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        AllianceConfig.alliance = AllianceConfig.Alliance.BLUE
    }

    override fun onStartButtonPressed() {
        // Starting pose mirrored for Blue Alliance
        follower.setStartingPose(Pose(20.900, 123.200, Math.toRadians(144.0)).mirror())
        buildPaths()

        val main = SequentialGroup(
            initCommands.farInit(),

            // Initial score
            FollowPath(paths[0], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            // Cycle 1
            ParallelRaceGroup(
                FollowPath(paths[1], true, 1.0),
                intakeAuto.autoIntakeNoGate(3.0)
            ),
            FollowPath(paths[2], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            // Cycle 2
            ParallelRaceGroup(
                FollowPath(paths[3], true, 1.0),
                intakeAuto.autoIntakeNoGate(2.0)
            ),
            FollowPath(paths[4], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            // Cycle 3
            ParallelRaceGroup(
                FollowPath(paths[5], true, 1.0),
                intakeAuto.autoIntakeNoGate(2.0)
            ),
            FollowPath(paths[6], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            // Final Leave/Park
            FollowPath(paths[7], true, 1.0)
        )
        main.schedule()
    }

    private fun buildPaths() {
        // Path 0: Initial Score
        val scorePose0 = follower.pathBuilder()
            .addPath(BezierLine(Pose(20.900, 123.200).mirror(), Pose(59.200, 84.000).mirror()))
            .setLinearHeadingInterpolation(Math.toRadians(144.0), Math.toRadians(180.0))
            .build()

        // Path 1: Intake 1
        val intake1 = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(59.200, 84.000).mirror(),
                Pose(50.700, 49.200).mirror(),
                Pose(16.300, 56.400).mirror(),
                Pose(13.000, 61.700).mirror()
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(144.0))
            .build()

        // Path 2: Score Pose 1
        val scorePose1 = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(13.000, 61.700).mirror(),
                Pose(43.119, 59.491).mirror(),
                Pose(58.984, 83.732).mirror()
            ))
            .setLinearHeadingInterpolation(Math.toRadians(144.0), Math.toRadians(180.0))
            .build()

        // Path 3: Intake 2
        val intake2 = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(58.984, 83.732).mirror(),
                Pose(50.900, 49.300).mirror(),
                Pose(16.200, 56.500).mirror(),
                Pose(12.086, 59.237).mirror()
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(144.0))
            .build()

        // Path 4: Score Pose 2
        val scorePose2 = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(12.086, 59.237).mirror(),
                Pose(43.170, 59.068).mirror(),
                Pose(59.300, 83.900).mirror()
            ))
            .setLinearHeadingInterpolation(Math.toRadians(144.0), Math.toRadians(180.0))
            .build()

        // Path 5: Intake 3
        val intake3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(59.300, 83.900).mirror(), Pose(16.702, 83.901).mirror()))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        // Path 6: Score Pose 3
        val scorePose3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(16.702, 83.901).mirror(), Pose(59.300, 84.200).mirror()))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()

        // Path 7: Leave
        val leave = follower.pathBuilder()
            .addPath(BezierLine(Pose(59.300, 84.200).mirror(), Pose(58.791, 57.837).mirror()))
            .setTangentHeadingInterpolation()
            .build()

        paths = arrayOf(
            scorePose0, intake1, scorePose1,
            intake2, scorePose2, intake3,
            scorePose3, leave
        )
    }

    override fun onUpdate() {
        Turret.currentState = Turret.State.LOCKED
        telemetry.addData("Pose", follower.pose)
        telemetry.update()
    }

    override fun onStop() {
        FlyWheel.setVelocity(0.0)
        Gate.setPosition(GateConstants.GATE_CLOSED)
        Intake.run(0.0)
        Hood.setPosition(HoodConstants.HOOD_CLOSE)
        Turret.stop()
        Drive.savePose()
    }
}