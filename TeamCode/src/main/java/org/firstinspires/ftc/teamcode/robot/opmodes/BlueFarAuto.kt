//package org.firstinspires.ftc.teamcode.robot.opmodes
//
//import PropDetector
//import PropZone
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import dev.turtles.anchor.component.stock.instant
//import dev.turtles.electriceel.opmode.AnchorOpMode
//import dev.turtles.lilypad.Button
//import dev.turtles.lilypad.impl.FTCGamepad
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
//import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
//import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
//import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
//import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
//import org.openftc.easyopencv.OpenCvCamera
//import org.openftc.easyopencv.OpenCvCameraFactory
//import org.openftc.easyopencv.OpenCvCameraRotation
//import org.openftc.easyopencv.OpenCvWebcam
//
//@Autonomous
//class BlueFarAuto : AnchorOpMode() {
//    lateinit var robot : Robot
//    lateinit var smec : ScoringMechanism
//    lateinit var drive : CenterstageMecanumDrive
//    lateinit var webcam : OpenCvWebcam
//    var detector = PropDetector(telemetry)
//    var zone = PropZone.UNKNOWN
//    override fun prerun() {
//        val driver = FTCGamepad(gamepad1)
//        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
//        smec = robot.scoringMechanism
//        drive = robot.driveBase.dt()
//        robot.init(this.world)
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
//        driver[Button.Key.DPAD_LEFT] onActivate instant {
//            smec.leftClawState = when (smec.leftClawState) {
//                ClawPositions.OPEN -> ClawPositions.CLOSED
//                ClawPositions.CLOSED -> {
//                    if (smec.state == ScoringMechanism.State.INTAKE)
//                        ClawPositions.OPEN
//                    else if (smec.state == ScoringMechanism.State.INTAKE )
//                        ClawPositions.OPEN
//                    else ClawPositions.DROP
//                }
//                ClawPositions.DROP -> ClawPositions.CLOSED
//            }
//        }
//        driver[Button.Key.DPAD_RIGHT] onActivate instant {
//            smec.rightClawState = when (smec.rightClawState) {
//                ClawPositions.OPEN -> ClawPositions.CLOSED
//                ClawPositions.DROP -> ClawPositions.CLOSED
//
//                ClawPositions.CLOSED -> {
//                    if (smec.state == ScoringMechanism.State.INTAKE )
//                        ClawPositions.OPEN
//                    else if (smec.state == ScoringMechanism.State.CLOSE_INTAKE )
//                        ClawPositions.OPEN
//                    else ClawPositions.DROP;
//                }
//            }
//        }
//        while (opModeInInit()) {
//
//        }
//    }
//
//    override fun run() {
//        zone = detector.zone
//        val startPose = Pose2d(32.0, -61.75, -90.0)
//
//        // Treat unknown as if it is right
//        val p1traj = when (zone) {
//            PropZone.CENTER -> drive.trajectoryBuilder(startPose)
//                .strafeTo(Vector2d(37.5, -60.0))
//                .build()
//            PropZone.RIGHT -> drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(Pose2d(-18.0, -34.0, 0.0))
//                .build()
//            PropZone.LEFT -> drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(Pose2d(-33.0, -41.5, 0.0))
//                .build()
//            PropZone.UNKNOWN -> drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(Pose2d(-18.0, -34.0, 0.0))
//                .build()
//        }
//        drive.followTrajectory(p1traj)
//        val p2traj = when (zone) {
//            PropZone.CENTER -> drive.trajectoryBuilder(p1traj.end())
//                .strafeTo(Vector2d(37.5, -12.0))
//                .build()
//            PropZone.RIGHT -> drive.trajectoryBuilder(p1traj.end())
//                .strafeTo(Vector2d(-12.0, -34.0))
//                .build()
//            PropZone.LEFT -> drive.trajectoryBuilder(p1traj.end())
//                .strafeTo(Vector2d(-33.0, -29.0))
//                .build()
//            PropZone.UNKNOWN -> drive.trajectoryBuilder(p1traj.end())
//                .strafeTo(Vector2d(-12.0, -34.0))
//                .build()
//        }
//
//        val p3traj = when (zone) {
//            PropZone.CENTER -> drive.trajectoryBuilder(p2traj.end())
//                .lineToLinearHeading(Pose2d(37.5, -12.0, 0.0))
//                .build()
//        }
//
//        val p4traj = when (zone) {
//            PropZone.CENTER -> drive.trajectoryBuilder(p3traj.end())
//                .strafeTo(Vector2d(-43.0, -12.0))
//                .build()
//        }
//
//        val p5traj = when (zone) {
//            PropZone.CENTER -> drive.trajectoryBuilder(p4traj.end())
//                .strafeTo(Vector2d(-43.0, -12.0))
//                .build()
//        }
//
//        drive.followTrajectory(p1traj)
//        drive.followTrajectory(p2traj)
//        drive.followTrajectory(p3traj)
//        drive.followTrajectory(p4traj)
//        drive.followTrajectory(p5traj)
//    }
//}
