package org.firstinspires.ftc.teamcode.robot.opmodes

import PropDetector
import PropZone
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.fasterxml.jackson.databind.annotation.JsonAppend.Prop
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.roadrunner.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.PI

@Autonomous
class BlueCloseAuto : AnchorOpMode() {
    lateinit var robot : Robot
    lateinit var smec : ScoringMechanism
    lateinit var drive : CenterstageMecanumDrive
    lateinit var webcam : OpenCvWebcam
    var detector = PropDetector(telemetry)
    var zone = PropZone.UNKNOWN
    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        smec = robot.scoringMechanism
        drive = robot.driveBase.dt()
        robot.elbow.isEnabled = true
        robot.init(this.world)
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "webcam1"
            ), cameraMonitorViewId
        )
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })
        webcam.setPipeline(detector)
        OpenBothClaw(robot.leftClaw, robot.rightClaw)

        driver[Button.Key.DPAD_LEFT] onActivate instant {
            smec.leftClawState = when (smec.leftClawState) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.CLOSED -> {
                    if (smec.state == ScoringMechanism.State.INTAKE)
                        ClawPositions.OPEN
                    else if (smec.state == ScoringMechanism.State.INTAKE )
                        ClawPositions.OPEN
                    else ClawPositions.DROP
                }
                ClawPositions.DROP -> ClawPositions.CLOSED
            }
        }
        driver[Button.Key.DPAD_RIGHT] onActivate instant {
            smec.rightClawState = when (smec.rightClawState) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.DROP -> ClawPositions.CLOSED

                ClawPositions.CLOSED -> {
                    if (smec.state == ScoringMechanism.State.INTAKE )
                        ClawPositions.OPEN
                    else if (smec.state == ScoringMechanism.State.CLOSE_INTAKE )
                        ClawPositions.OPEN
                    else ClawPositions.DROP;
                }
            }
        }
    }

    override fun run() {
        webcam.stopStreaming()
        zone = detector.zone
        val startPose = Pose2d(0.0, 0.0, 0.0)

        // Treat unknown as if it is right
        val p1traj = when (zone) {
            PropZone.CENTER -> drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(-24.0, -11.0, PI / 2))
                .build()

            PropZone.RIGHT -> drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(-35.0, -18.0, 0.0))
                .build()

            PropZone.LEFT -> drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(0.0, 0.0, 0.0))
                .build()

            PropZone.UNKNOWN -> drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(0.0, 0.0, 0.0))
                .build()
        }


        val p2traj = when (zone) {
            PropZone.CENTER -> drive.trajectoryBuilder(p1traj.end())
                .strafeTo(Vector2d(-36.0, -11.0))
                .build()
            PropZone.RIGHT -> drive.trajectoryBuilder(p1traj.end())
                .strafeTo(Vector2d(-12.0, -3.0))
                .build()
            PropZone.LEFT -> drive.trajectoryBuilder(p1traj.end())
                .strafeTo(Vector2d(-33.0, -29.0))
                .build()
            PropZone.UNKNOWN -> drive.trajectoryBuilder(p1traj.end())
                .strafeTo(Vector2d(-12.0, -34.0))
                .build()
        }
//
        val p3traj = drive.trajectoryBuilder(p2traj.end())
            .strafeTo(Vector2d(-24.0, -20.0))
            .build()

//        val follower = TrajectoryFollower(drive, drive.trajectoryBuilder(Pose2d(0.0, 0.0, -PI / 2))
//            .lineToLinearHeading(Pose2d(0.0, 24.0, Math.toRadians(0z0./**/0)))
//            .build());
        val p1follower = TrajectoryFollower(drive, p1traj)
        val p2follower = TrajectoryFollower(drive, p2traj)
        val p3follower = TrajectoryFollower(drive, p3traj)
        + series (
            p1follower,
            parallel (
                instant {
                    smec.state = ScoringMechanism.State.CLOSE_INTAKE
                },
                p2follower,
            ),
            instant {
                smec.leftClawState = ClawPositions.OPEN
            },
            p3follower,
            instant {
                smec.pixelHeight = 1.0
                smec.state = ScoringMechanism.State.DEPOSIT
            },
            delay(2.0),
            instant {
                smec.rightClawState = ClawPositions.OPEN
            }
        )


    }
}