package org.firstinspires.ftc.teamcode.robot.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import org.firstinspires.ftc.teamcode.roadrunner.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.PI


@Autonomous
class SimpleAutoTest: AnchorOpMode() {
    lateinit var robot : Robot
    lateinit var drive : CenterstageMecanumDrive

    override fun prerun() {
        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        robot.init(world)
        drive = robot.driveBase.dt()
    }

    override fun run() {
//        val startPose = Pose2d(33.0, -61.75, -PI/2)
        val trajectory = drive.trajectoryBuilder(Pose2d())
            .forward(12.0)
            .build()
        val follower = TrajectoryFollower(drive, trajectory)

        + series(follower, follower)
    }

}