package org.firstinspires.ftc.teamcode.robot.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import org.firstinspires.ftc.teamcode.roadrunner.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import kotlin.math.PI

@Autonomous
class IMUtesting : AnchorOpMode() {
    lateinit var robot : Robot
    lateinit var drive : CenterstageMecanumDrive
    lateinit var smec : ScoringMechanism
    val startPose = Pose2d(IMUtestconfig.startX, IMUtestconfig.startY, IMUtestconfig.startHeading)
    override fun prerun() {
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, startPose = startPose)
        drive = robot.driveBase.dt
        smec = robot.scoringMechanism
        robot.init(this.world)
        robot.scoringMechanism.state = ScoringMechanism.State.TRAVEL
//        smec.state = ScoringMechanism.State.DEPOSIT
    }

    override fun run() {
        val pose1 = Pose2d(IMUtestconfig.coords1[0], IMUtestconfig.coords1[1], IMUtestconfig.startHeading)
        val runner = drive.trajectorySequenceRunner


        val seq = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(Pose2d(IMUtestconfig.coords1[0], IMUtestconfig.coords1[1], IMUtestconfig.coords1[2]))
            .build()

        val follower = TrajectoryFollower(drive, seq)
        + series(
//            follower,
            instant {smec.state = ScoringMechanism.State.DEPOSIT},
            delay(3.0)
        )
    }
}