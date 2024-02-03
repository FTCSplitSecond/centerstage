package org.firstinspires.ftc.teamcode.robot.opmodes

import PropDetector
import PropZone
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
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
import org.firstinspires.ftc.teamcode.claw.commands.SetLeftClawState
import org.firstinspires.ftc.teamcode.claw.commands.SetRightClawState
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.roadrunner.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.AutoConfig
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
        drive = robot.driveBase.dt
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
            val leftClawState = when (smec.leftClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.CLOSED -> {
                if (smec.armState == ScoringMechanism.State.INTAKE)
                    ClawPositions.OPEN
                else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE )
                    ClawPositions.OPEN
                else ClawPositions.DROP
            }
                ClawPositions.DROP -> ClawPositions.CLOSED
            }
            SetLeftClawState(smec.leftClaw, leftClawState)
        }

        driver[Button.Key.DPAD_RIGHT] onActivate run {
            val rightClawState = when (smec.rightClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.DROP -> ClawPositions.CLOSED

                ClawPositions.CLOSED -> {
                    if (smec.armState == ScoringMechanism.State.INTAKE )
                        ClawPositions.OPEN
                    else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE )
                        ClawPositions.OPEN
                    else ClawPositions.DROP;
                }
            }
            SetRightClawState(smec.rightClaw, rightClawState)
        }
    }

    override fun run() {
        webcam.stopStreaming()
        zone = detector.zone
        val startPose = Pose2d(0.0, 0.0, 0.0)

        // Treat unknown as if it is right
        zone = when (AutoConfig.MODE) {
            0 -> PropZone.CENTER
            1 -> PropZone.RIGHT
            2 -> PropZone.LEFT
            else -> zone
        }
        if (zone == PropZone.UNKNOWN) {
            zone = PropZone.CENTER
        }
        // Move to spike mark
        val p1traj = when (zone) {
            PropZone.CENTER -> drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(AutoConfig.BLUE_CLOSE_CENTER_X[0], AutoConfig.BLUE_CLOSE_CENTER_Y[0], PI / 2))
                .build()

            PropZone.RIGHT -> drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(AutoConfig.BLUE_CLOSE_RIGHT_X[0], AutoConfig.BLUE_CLOSE_RIGHT_Y[0], PI/2))
                .build()

            PropZone.LEFT -> drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(Pose2d(AutoConfig.BLUE_CLOSE_LEFT_X[0], AutoConfig.BLUE_CLOSE_LEFT_Y[0], PI/2))
                .build()
            else -> TODO()
        }

        // Move to purple pixel deposit
        val p2traj = when (zone) {
            PropZone.CENTER -> drive.trajectoryBuilder(p1traj.end())
                .strafeTo(Vector2d(AutoConfig.BLUE_CLOSE_CENTER_X[1], AutoConfig.BLUE_CLOSE_CENTER_Y[1]))
                .build()
            PropZone.RIGHT -> drive.trajectoryBuilder(p1traj.end())
                .strafeTo(Vector2d(AutoConfig.BLUE_CLOSE_RIGHT_X[1], AutoConfig.BLUE_CLOSE_RIGHT_Y[1]))
                .build()
            PropZone.LEFT -> drive.trajectoryBuilder(p1traj.end())
                .strafeTo(Vector2d(AutoConfig.BLUE_CLOSE_LEFT_X[1], AutoConfig.BLUE_CLOSE_LEFT_Y[1]))
                .build()
            else -> TODO()
        }

        // Move to yellow pixel deposit
        val p3traj = when (zone) {
            PropZone.CENTER -> drive.trajectoryBuilder(p2traj.end())
                .strafeTo(Vector2d(AutoConfig.BACKDROP_CENTER_X, AutoConfig.BLUE_CLOSE_CENTER_Y[2]))// TODO: MOVE CLOSER TO BACKDROP (-y)
                .build()
            PropZone.RIGHT -> drive.trajectoryBuilder(p2traj.end())
                .strafeTo(Vector2d(AutoConfig.BACKDROP_RIGHT_X, AutoConfig.BLUE_CLOSE_RIGHT_Y[2]))// TODO: MOVE CLOSER TO BACKDROP (-y)
                .build()
            PropZone.LEFT -> drive.trajectoryBuilder(p2traj.end())
                .strafeTo(Vector2d(AutoConfig.BACKDROP_LEFT_X, AutoConfig.BLUE_CLOSE_LEFT_Y[2]))// TODO: MOVE CLOSER TO BACKDROP (-y)
                .build()
            else -> TODO()
        }

//        // Move to park
//        val p4traj =


        val p1follower = TrajectoryFollower(drive, p1traj)
        val p2follower = TrajectoryFollower(drive, p2traj)
        val p3follower = TrajectoryFollower(drive, p3traj)
        + series (
            p1follower,
            parallel (
                instant {
                    smec.armState = ScoringMechanism.State.CLOSE_INTAKE
                },
                p2follower,
            ),
            instant {
                smec.leftClawState = ClawPositions.OPEN
            },
            parallel (
                instant {
                    smec.armState = ScoringMechanism.State.TRAVEL
                },
                p3follower
            ),
            instant {
                smec.pixelHeight = 0.0
                smec.armState = ScoringMechanism.State.DEPOSIT
            },
            delay(2.0),
            instant {
                smec.rightClawState = ClawPositions.OPEN
            },
            delay(0.5),
            instant {
                smec.armState = ScoringMechanism.State.DROP
            },
            delay(0.5),
            instant {
                smec.armState = ScoringMechanism.State.TRAVEL
            }
        )


    }
}