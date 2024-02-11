package org.firstinspires.ftc.teamcode.robot.testopmodes

import PropZone
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.roadrunner.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.vision.processors.PixelStackDetector
import org.openftc.easyopencv.OpenCvWebcam

@Autonomous
class PixelStackTester : AnchorOpMode() {
    lateinit var robot : Robot
    lateinit var smec : ScoringMechanism
    lateinit var drive : CenterstageMecanumDrive
    lateinit var webcam : OpenCvWebcam
    var detector = PixelStackDetector(telemetry)
    var zone = PropZone.UNKNOWN
    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        smec = robot.scoringMechanism
        drive = robot.driveBase.dt
        robot.elbow.isEnabled = true
        robot.init(this.world)
        OpenBothClaw(robot.leftClaw, robot.rightClaw)
//        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
//            "cameraMonitorViewId",
//            "id",
//            hardwareMap.appContext.packageName
//        )
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(
//            hardwareMap.get(
//                WebcamName::class.java, "webcam1"
//            ), cameraMonitorViewId
//        )
//        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
//            override fun onOpened() {
//                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT)
//            }
//
//            override fun onError(errorCode: Int) {}
//        })
//        webcam.setPipeline(detector)
//        OpenBothClaw(robot.leftClaw, robot.rightClaw)
    }

    override fun run() {
//        webcam.stopStreaming()
        val traj = drive.trajectoryBuilder(Pose2d())
            .forward(2.0)
            .build()
        val follower = TrajectoryFollower(drive, traj)
        + series (
                smec.setArmState(ScoringMechanism.State.STACK_INTAKE),
            follower,
            instant {
                robot.leftClaw.position = ClawPositions.CLOSED
            },
                smec.setArmState(ScoringMechanism.State.TRAVEL)
        )
    }
}