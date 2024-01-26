package org.firstinspires.ftc.teamcode.robot.opmodes

import PropDetector
import PropZone
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.robot.subsystems.Alliance
import org.firstinspires.ftc.teamcode.robot.subsystems.AutoConfig
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.PI

@Autonomous
class BlueFarAuto : AnchorOpMode() {
    lateinit var robot : Robot
    lateinit var smec : ScoringMechanism
    lateinit var drive : CenterstageMecanumDrive
    lateinit var webcam : OpenCvWebcam
    var detector = PropDetector(telemetry)
    var zone = PropZone.UNKNOWN
    val startPose = Pose2d(-32.0, 62.0, PI/2)
    override fun prerun() {
        val driver = FTCGamepad(gamepad1)

        Robot.alliance = Alliance.BLUE

        robot = Robot(hardwareMap, this.hardwareManager, telemetry, startPose = startPose)
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
        val runner = drive.trajectorySequenceRunner
        // set testing zones
        zone = when (AutoConfig.MODE) {
            0 -> PropZone.CENTER
            1 -> PropZone.RIGHT
            2 -> PropZone.LEFT
            else -> zone
        }
        if (zone == PropZone.UNKNOWN) {
            zone = PropZone.CENTER
        }
        // Treat unknown as if it is right
        lateinit var t1 : Trajectory
        lateinit var t2 : Trajectory
        lateinit var t2_5 : Trajectory
        lateinit var t3 : Trajectory
        lateinit var t4 : Trajectory
        lateinit var t5 : Trajectory
        lateinit var t6 : Trajectory
        lateinit var t7 : Trajectory
        when (zone) {
            PropZone.CENTER -> {
                t1 = drive.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_CENTER_X[0], AutoConfig.BLUE_FAR_CENTER_Y[0]))
                    .build()
                t2 = drive.trajectoryBuilder(t1.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_CENTER_X[1], AutoConfig.BLUE_FAR_CENTER_Y[1]))
                    .build()
                t3 = drive.trajectoryBuilder(t2.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_CENTER_X[2], AutoConfig.BLUE_FAR_CENTER_Y[2]))
                    .build()
                t4 = drive.trajectoryBuilder(t3.end())
                    .lineToLinearHeading(Pose2d(AutoConfig.BLUE_FAR_CENTER_X[3], AutoConfig.BLUE_FAR_CENTER_Y[3], PI))
                    .build()
                t5 = drive.trajectoryBuilder(t4.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_CENTER_X[4], AutoConfig.BLUE_FAR_CENTER_Y[4]))
                    .build()
                t6 = drive.trajectoryBuilder(t5.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_CENTER_X[5], AutoConfig.BLUE_FAR_CENTER_Y[5]))
                    .build()
                t7 = drive.trajectoryBuilder(t6.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_CENTER_X[6], AutoConfig.BLUE_FAR_CENTER_Y[6]))
                    .build()
            }
            PropZone.RIGHT -> {
                t1 = drive.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_RIGHT_X[0], AutoConfig.BLUE_FAR_RIGHT_Y[0]))
                    .build()
                t2 = drive.trajectoryBuilder(t1.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_RIGHT_X[1], AutoConfig.BLUE_FAR_RIGHT_Y[1]))
                    .build()
                t3 = drive.trajectoryBuilder(t2.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_RIGHT_X[2], AutoConfig.BLUE_FAR_RIGHT_Y[2]))
                    .build()
                t4 = drive.trajectoryBuilder(t3.end())
                    .lineToLinearHeading(Pose2d(AutoConfig.BLUE_FAR_RIGHT_X[3], AutoConfig.BLUE_FAR_RIGHT_Y[3], PI))
                    .build()
                t5 = drive.trajectoryBuilder(t4.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_RIGHT_X[4], AutoConfig.BLUE_FAR_RIGHT_Y[4]))
                    .build()
                t6 = drive.trajectoryBuilder(t5.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_RIGHT_X[5], AutoConfig.BLUE_FAR_RIGHT_Y[5]))
                    .build()
                t7 = drive.trajectoryBuilder(t6.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_RIGHT_X[6], AutoConfig.BLUE_FAR_RIGHT_Y[6]))
                    .build()
            }
            PropZone.LEFT -> {
                t1 = drive.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[0], AutoConfig.BLUE_FAR_LEFT_Y[0]))
                    .build()
                t2 = drive.trajectoryBuilder(t1.end())
                    .lineToLinearHeading(Pose2d(AutoConfig.BLUE_FAR_LEFT_X[1], AutoConfig.BLUE_FAR_LEFT_Y[1], 0.0))
                    .build()
                t2_5 = drive.trajectoryBuilder(t2.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[2], AutoConfig.BLUE_FAR_LEFT_Y[2]))
                    .build()
                t3 = drive.trajectoryBuilder(t2_5.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[3], AutoConfig.BLUE_FAR_LEFT_Y[3]))
                    .build()
                t4 = drive.trajectoryBuilder(t3.end())
                    .lineToLinearHeading(Pose2d(AutoConfig.BLUE_FAR_LEFT_X[4], AutoConfig.BLUE_FAR_LEFT_Y[4], PI))
                    .build()
                t5 = drive.trajectoryBuilder(t4.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[5], AutoConfig.BLUE_FAR_LEFT_Y[5]))
                    .build()
                t6 = drive.trajectoryBuilder(t5.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[6], AutoConfig.BLUE_FAR_LEFT_Y[6]))
                    .build()
                t7 = drive.trajectoryBuilder(t6.end())
                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[7], AutoConfig.BLUE_FAR_LEFT_Y[7]))
                    .build()
            }
            else -> TODO()
        }

        val t1follower = TrajectoryFollower(drive, t1)
        val t2follower = TrajectoryFollower(drive, t2)
        val t2_5follower = TrajectoryFollower(drive, t2_5)
        val t3follower = TrajectoryFollower(drive, t3)
        val t4follower = TrajectoryFollower(drive, t4)
        val t5follower = TrajectoryFollower(drive, t5)
        val t6follower = TrajectoryFollower(drive, t6)
        val t7follower = TrajectoryFollower(drive, t7)
        + series (
            t1follower,
            parallel(
                series(
                    delay(1.0),
                    instant {smec.state = ScoringMechanism.State.CLOSE_INTAKE},
                ),
                t2follower,
            ),
            instant {smec.leftClawState = ClawPositions.OPEN},
            parallel(
                instant {smec.state = ScoringMechanism.State.TRAVEL},
                // TODO: we want to follow only trajectory t3 UNLESS we are on the left path, in which case we have to follow t2_5 and t3
                instant {
                    if (zone == PropZone.LEFT) {
                        t2_5follower
                    } else {
                        t3follower
                    }
                }
            ),
            // TODO: these are all of the steps for after we reach the backdrop side of the field
            // TODO: should be more or less the same for each randomization
//            parallel(
//                instant {smec.state = ScoringMechanism.State.DEPOSIT},
//                t4follower
//            ),
//            t5follower,
//            instant {smec.rightClawState = ClawPositions.OPEN},
//            t6follower,
//            instant {smec.state = ScoringMechanism.State.TRAVEL},
//            t7follower
        )


    }
}