package org.firstinspires.ftc.teamcode.robot.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.electriceel.opmode.AnchorOpMode
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism

@Autonomous
class IMUtesting : AnchorOpMode() {
    lateinit var robot : Robot
    lateinit var drive : CenterstageMecanumDrive
    override fun prerun() {
        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        drive = robot.driveBase.dt
        robot.init(this.world)
        robot.scoringMechanism.state = ScoringMechanism.State.TRAVEL
    }

    override fun run() {
//        drive.IMU_OFFSET = IMUtestconfig.imuOffset
        val startPose = Pose2d(IMUtestconfig.startX, IMUtestconfig.startY, IMUtestconfig.startHeading)
        val pose1 = Pose2d(IMUtestconfig.coords1[0], IMUtestconfig.coords1[1], IMUtestconfig.startHeading)
        val runner = drive.trajectorySequenceRunner
        val p1 = drive.trajectoryBuilder(startPose)
//            .strafeTo(Vector2d(24.0, 60.0))
            .lineToLinearHeading(pose1)
            .build()

        val seq = drive.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(Pose2d(IMUtestconfig.coords1[0], IMUtestconfig.coords1[1], IMUtestconfig.startHeading))
            .build()

        runner.followTrajectorySequenceAsync(seq)
    }
}