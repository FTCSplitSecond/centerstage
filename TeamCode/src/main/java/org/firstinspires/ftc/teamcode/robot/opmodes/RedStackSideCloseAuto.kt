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
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.mecanum.commands.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.robot.util.Alliance
import org.firstinspires.ftc.teamcode.robot.util.adjustForAlliance
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.PI


@Autonomous
class RedStackSideCloseAuto : AnchorOpMode() {
    lateinit var robot: Robot
    lateinit var smec: ScoringMechanism
    lateinit var drive: CenterstageMecanumDrive
    lateinit var webcam: OpenCvWebcam
    var detector = PropDetector(telemetry)
    val startPose = Pose2d(-.0, 63.0, PI / 2)
    val alliance = Alliance.RED

    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        Robot.alliance = alliance
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

//        +OpenBothClaw(robot.leftClaw, robot.rightClaw)
        +smec.setDepositPixelLevel(-0.5)

        driver[Button.Key.DPAD_LEFT] onActivate instant {
            robot.leftClaw.position = when (robot.leftClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.CLOSED -> {
                    if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE)
                        ClawPositions.OPEN
                    else if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE)
                        ClawPositions.OPEN
                    else ClawPositions.DROP
                }

                ClawPositions.DROP -> ClawPositions.CLOSED
            }
        }
        driver[Button.Key.DPAD_RIGHT] onActivate instant {
            robot.rightClaw.position = when (robot.rightClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.DROP -> ClawPositions.CLOSED

                ClawPositions.CLOSED -> {
                    if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE)
                        ClawPositions.OPEN
                    else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
                        ClawPositions.OPEN
                    else ClawPositions.DROP;
                }
            }
        }
    }

    fun getAllianceHeading(alliance: Alliance): Double {
        return when (alliance) {
            Alliance.RED -> -PI / 2
            Alliance.BLUE -> PI / 2
        }
    }

    override fun run() {
        val zoneDetected = detector.zone
        webcam.stopStreaming()

        val spinOffset = when (alliance) {
            Alliance.RED -> 0.0001
            Alliance.BLUE -> -0.0001
        }
//        val startHeading = getAllianceHeading(alliance)
//        val startPose = Pose2d(-32.0, 62.0, startHeading).adjustForAlliance(alliance)
        val awayFromWallPosition = Vector2d(-42.0, 45.0).adjustForAlliance(alliance)

        val purplePixelPoseBackdropSide =
            Pose2d(Vector2d(-38.0, 51.0), (startPose.heading+PI/5.5)).adjustForAlliance(alliance)
        val purplePixelPoseCenter =
            Pose2d(Vector2d(-40.0, 50.5), startPose.heading).adjustForAlliance(alliance)
        val purplePixelPoseAwayFromBackdrop =
            Pose2d(Vector2d(-38.0, 51.0), (startPose.heading-PI/7)).adjustForAlliance(alliance)
        val purplePixelPose = when (zoneDetected) {
            PropZone.RIGHT -> if (alliance == Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
            PropZone.CENTER, PropZone.UNKNOWN -> purplePixelPoseCenter
            PropZone.LEFT -> if (alliance == Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
        }

        val stackAxis = Pose2d(Vector2d(-48.0, 35.0), PI).adjustForAlliance(alliance)
        val stackLocation = Vector2d(-57.7, 35.0).adjustForAlliance(alliance)

        val transitLaneY = 58.0
        val nearBackDropLaneX = 40.0
        val backDropScoreX = 43.0

        val transitLanePoseAfterPurplePixel =
            (Vector2d(-44.0, transitLaneY)).adjustForAlliance(alliance)
        val transitLaneBackDropSide =
            Vector2d(nearBackDropLaneX-20.0, transitLaneY).adjustForAlliance(alliance)
        val transitLanePixelStackSide =
            Vector2d(-48.0, transitLaneY).adjustForAlliance(alliance)

        val backDropScoringClawOffset = 0.0 // offset to help pixels land better if needed
        val backDropZoneSpacing = 6.0
        val backDropCenterY = 36.0
        val nearBackDropCenter = Vector2d(
            nearBackDropLaneX,
            backDropCenterY + backDropScoringClawOffset
        ).adjustForAlliance(alliance)
        val nearBackDropLeft =
            Vector2d(nearBackDropLaneX, nearBackDropCenter.y + backDropZoneSpacing).adjustForAlliance(alliance)
        val nearBackDropRight =
//            Vector2d(nearBackDropLaneX, nearBackDropCenter.y - backDropZoneSpacing)
            Vector2d(nearBackDropLaneX, 30.0).adjustForAlliance(alliance)
        val nearBackDropPosition = when (zoneDetected) {
            PropZone.LEFT -> nearBackDropLeft
            PropZone.CENTER, PropZone.UNKNOWN -> nearBackDropCenter
            PropZone.RIGHT -> nearBackDropRight
        }
        val dropWhitePosition = Vector2d(
            backDropScoreX,
            38.0
        ).adjustForAlliance(alliance)
        val backDropScoringPosition = Vector2d(
            backDropScoreX,
            nearBackDropPosition.y
        )  // no need to adjust for alliance (already there)

        val parkInsidePosition = Vector2d(48.0, transitLaneY).adjustForAlliance(alliance)
        val parkOutsidePosition = Vector2d(backDropScoreX, 60.0).adjustForAlliance(alliance)


        // trajectories
        val moveAwayFromWallTrajectory = drive.trajectoryBuilder(startPose)
            .lineTo(awayFromWallPosition)
            .build()
        val moveToScorePurplePixelTrajectory =
            drive.trajectoryBuilder(moveAwayFromWallTrajectory.end())
                .lineToLinearHeading(purplePixelPose)
                .build()
        val moveToAlignWithStackTrajectory =
            drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
                .lineToLinearHeading(stackAxis)
                .build()
        val moveToStackTrajectory = drive.trajectoryBuilder(moveToAlignWithStackTrajectory.end())
            .lineTo(stackLocation)
            .build()
        val backAwayFromStackTrajectory =
            drive.trajectoryBuilder(moveToStackTrajectory.end())
                .lineToLinearHeading(stackAxis)
                .build()
        // UNUSED W/ WHITE PIXEL
        val moveToCloseTransitLaneTrajectory =
            drive.trajectoryBuilder(moveToStackTrajectory.end())
                .splineToConstantHeading(transitLanePoseAfterPurplePixel, 0.0)
                .build()
        val moveToBackDropLaneTrajectory =
            drive.trajectoryBuilder(moveToCloseTransitLaneTrajectory.end())
                .lineToConstantHeading(transitLaneBackDropSide)
                .build()
        val moveToNearBackdropTrajectory =
            drive.trajectoryBuilder(moveToBackDropLaneTrajectory.end())
                .splineToConstantHeading(nearBackDropPosition, 0.0)
                .build()
        val moveToBackdropTrajectory =
            drive.trajectoryBuilder(moveToAlignWithStackTrajectory.end(), true)
                .splineToConstantHeading(transitLanePoseAfterPurplePixel, 0.0)
                .splineToConstantHeading(transitLaneBackDropSide,0.0) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
                .splineToConstantHeading(nearBackDropPosition, 0.0)
                .build()
        val moveToScoreBackDropTrajectory =
            drive.trajectoryBuilder(moveToNearBackdropTrajectory.end())
                .lineTo(backDropScoringPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
                .build()
        val moveToDropWhiteTrajectory =
            drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
                .lineTo(dropWhitePosition)
                .build()
        val backAwayFromBackDropTrajectory =
            drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
                .lineTo(nearBackDropPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
                .build()
        val moveToTransitLaneToPixelStacksTrajectory =
            drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
                .lineTo(transitLaneBackDropSide)
                .build()
        val moveToStackLineTrajectory =
            drive.trajectoryBuilder(moveToTransitLaneToPixelStacksTrajectory.end())
                .splineToConstantHeading(transitLanePoseAfterPurplePixel, 0.0)
                .build()
        val moveToPickUpFromStackTrajectory =
            drive.trajectoryBuilder(moveToStackLineTrajectory.end())
                .lineToLinearHeading(stackAxis)
                .build()

//        val moveToTransitLaneToPixelStacksTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
//                .lineTo(transitLaneBackDropSide)
//                .lineTo(transitLanePixelStackSide)
//                .build()
//        val moveToTransitLaneFromPixelStacksTrajectory = drive.trajectoryBuilder(moveToTransitLaneToPixelStacksTrajectory.end())
//                .lineTo(transitLaneBackDropSide)
//                .build()

        val parkInsideTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(parkInsidePosition)
            .build()

        val parkOutsideTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(parkOutsidePosition)
            .build()

        // commands
        val moveAwayFromWall = TrajectoryFollower(drive, moveAwayFromWallTrajectory)
        val sendArmToDropPurple = smec.setArmState(ScoringMechanism.State.PURPLE_DROP)
        val moveToCloseIntake = smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
        val alignWithStack = TrajectoryFollower(drive, moveToAlignWithStackTrajectory)
        val moveToStack = TrajectoryFollower(drive, moveToStackTrajectory)
        val backAwayFromStack = TrajectoryFollower(drive, backAwayFromStackTrajectory)
        val moveToScorePurplePixel = TrajectoryFollower(drive, moveToScorePurplePixelTrajectory)
        val scorePurplePixel = instant { robot.rightClaw.position= ClawPositions.OPEN }
        val moveToTravel = smec.setArmState(ScoringMechanism.State.TRAVEL)
        val moveToCloseTransitLane = TrajectoryFollower(drive, moveToCloseTransitLaneTrajectory)
        val moveToBackDropLane = TrajectoryFollower(drive, moveToBackDropLaneTrajectory)
        val moveToBackDrop = TrajectoryFollower(drive, moveToBackdropTrajectory)

        val moveToDropWhite = TrajectoryFollower(drive, moveToDropWhiteTrajectory)
        val moveToNearBackdrop = TrajectoryFollower(drive, moveToNearBackdropTrajectory)
        val moveToTransitLaneToPixelStacks = TrajectoryFollower(drive, moveToTransitLaneToPixelStacksTrajectory)
        val moveToStackLane = TrajectoryFollower(drive, moveToStackLineTrajectory)
        val moveToPickUpFromStack = TrajectoryFollower(drive, moveToPickUpFromStackTrajectory)
        val moveToDeposit = parallel(
            smec.setArmState(ScoringMechanism.State.DEPOSIT)
        )

        val moveToScoreBackDrop = TrajectoryFollower(drive, moveToScoreBackDropTrajectory)
        val backAwayFromBackDrop = TrajectoryFollower(drive, backAwayFromBackDropTrajectory)


        // used for extra cycles
//        val moveToTransitLaneToPixelStacks = TrajectoryFollower(drive, moveToTransitLaneToPixelStacksTrajectory)
//        val intakePixelsFromStack = instant {  } // add pixel intake here returns to transit lane when done
//        val moveToTransitLaneFromPixelStacks = TrajectoryFollower(drive, moveToTransitLaneFromPixelStacksTrajectory)

        val parkInside = TrajectoryFollower(drive, parkInsideTrajectory)
        val parkOutside = TrajectoryFollower(drive, parkOutsideTrajectory)
        val relocalizeFromAprilTags = instant { } // add april tag relocalization here

        // Now we schedule the commands
        +series(

            parallel(
                moveAwayFromWall, sendArmToDropPurple
            ),
            moveToScorePurplePixel,

            scorePurplePixel,

            parallel(
            smec.setArmState(ScoringMechanism.State.STACK_INTAKE_CLOSE),
            series(
                delay(0.5),
                alignWithStack
            )
            ),
            moveToStack,
            instant { robot.rightClaw.position = ClawPositions.CLOSED},
            backAwayFromStack,


            parallel(
                smec.setArmState(ScoringMechanism.State.TRAVEL),
                moveToBackDrop
            ),
            smec.setDepositPixelLevel(-0.5),
            parallel(
                smec.setArmState(ScoringMechanism.State.DEPOSIT),
                relocalizeFromAprilTags
            ),

            moveToScoreBackDrop,

            series(
                instant {robot.leftClaw.position = ClawPositions.DROP},
                instant {robot.rightClaw.position = ClawPositions.OPEN}
            ),

            backAwayFromBackDrop,

            smec.setArmState(ScoringMechanism.State.TRAVEL, ScoringMechanism.State.DEPOSIT),

            parkInside


            // add extra cycles here to the pixel stack
//            moveToTransitLaneToPixelStacks,
//            moveToStackLane,
//            moveToPickUpFromStack,
//            smec.setArmState(ScoringMechanism.State.STACK_INTAKE_CLOSE),
//            moveToStack,
//            instant {robot.leftClaw.position = ClawPositions.CLOSED},
//            parallel(
//                smec.setArmState(ScoringMechanism.State.TRAVEL),
//                moveToCloseTransitLane
//            ),
//            moveToBackDropLane,
//            parallel(
//                series(
//                    smec.setDepositPixelLevel(5.0),
//                    smec.setArmState(ScoringMechanism.State.DEPOSIT)
//                ),
//                moveToNearBackdrop,
//                relocalizeFromAprilTags,
//            ),
//            moveToScoreBackDrop,
//            instant { robot.rightClaw.position = ClawPositions.OPEN },
//            backAwayFromBackDrop
//            )


            // add extra cycles here to the pixel stack
//            moveToTransitLaneToPixelStacks,
//            intakePixelsFromStack,
//            moveToTransitLaneFromPixelStacks,
//
//            parallel(moveToNearBackdrop, moveToDeposit),
//            relocalizeFromApriltags,
//
//            moveToScoreBackDrop,
//            scoreBackDrop,
//            backAwayFromBackDrop,
//
//            parallel(moveToTravel, parkInside)

//        lateinit var t1: Trajectory
//        lateinit var t2: Trajectory
//        lateinit var t2_5: Trajectory
//        lateinit var t3: Trajectory
//        lateinit var t4: Trajectory
//        lateinit var t5: Trajectory
//        lateinit var t6: Trajectory
//        lateinit var t7: Trajectory
//        when (zone) {
//            // if UNKNOWN default to CENTER
//            PropZone.CENTER, PropZone.UNKNOWN -> {
//                t1 = drive.trajectoryBuilder(startPose)
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_CENTER_X[0],
//                            AutoConfig.BLUE_FAR_CENTER_Y[0]
//                        )
//                    )
//                    .build()
//                t2 = drive.trajectoryBuilder(t1.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_CENTER_X[1],
//                            AutoConfig.BLUE_FAR_CENTER_Y[1]
//                        )
//                    )
//                    .build()
//                t3 = drive.trajectoryBuilder(t2.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_CENTER_X[2],
//                            AutoConfig.BLUE_FAR_CENTER_Y[2]
//                        )
//                    )
//                    .build()
//                t4 = drive.trajectoryBuilder(t3.end())
//                    .lineToLinearHeading(
//                        Pose2d(
//                            AutoConfig.BLUE_FAR_CENTER_X[3],
//                            AutoConfig.BLUE_FAR_CENTER_Y[3],
//                            PI
//                        )
//                    )
//                    .build()
//                t5 = drive.trajectoryBuilder(t4.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_CENTER_X[4],
//                            AutoConfig.BLUE_FAR_CENTER_Y[4]
//                        )
//                    )
//                    .build()
//                t6 = drive.trajectoryBuilder(t5.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_CENTER_X[5],
//                            AutoConfig.BLUE_FAR_CENTER_Y[5]
//                        )
//                    )
//                    .build()
//                t7 = drive.trajectoryBuilder(t6.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_CENTER_X[6],
//                            AutoConfig.BLUE_FAR_CENTER_Y[6]
//                        )
//                    )
//                    .build()
//            }
//
//            PropZone.RIGHT -> {
//                t1 = drive.trajectoryBuilder(startPose)
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_RIGHT_X[0],
//                            AutoConfig.BLUE_FAR_RIGHT_Y[0]
//                        )
//                    )
//                    .build()
//                t2 = drive.trajectoryBuilder(t1.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_RIGHT_X[1],
//                            AutoConfig.BLUE_FAR_RIGHT_Y[1]
//                        )
//                    )
//                    .build()
//                t3 = drive.trajectoryBuilder(t2.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_RIGHT_X[2],
//                            AutoConfig.BLUE_FAR_RIGHT_Y[2]
//                        )
//                    )
//                    .build()
//                t4 = drive.trajectoryBuilder(t3.end())
//                    .lineToLinearHeading(
//                        Pose2d(
//                            AutoConfig.BLUE_FAR_RIGHT_X[3],
//                            AutoConfig.BLUE_FAR_RIGHT_Y[3],
//                            PI
//                        )
//                    )
//                    .build()
//                t5 = drive.trajectoryBuilder(t4.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_RIGHT_X[4],
//                            AutoConfig.BLUE_FAR_RIGHT_Y[4]
//                        )
//                    )
//                    .build()
//                t6 = drive.trajectoryBuilder(t5.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_RIGHT_X[5],
//                            AutoConfig.BLUE_FAR_RIGHT_Y[5]
//                        )
//                    )
//                    .build()
//                t7 = drive.trajectoryBuilder(t6.end())
//                    .lineTo(
//                        Vector2d(
//                            AutoConfig.BLUE_FAR_RIGHT_X[6],
//                            AutoConfig.BLUE_FAR_RIGHT_Y[6]
//                        )
//                    )
//                    .build()
//            }
//
//            PropZone.LEFT -> {
//                t1 = drive.trajectoryBuilder(startPose)
//                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[0], AutoConfig.BLUE_FAR_LEFT_Y[0]))
//                    .build()
//                t2 = drive.trajectoryBuilder(t1.end())
//                    .lineToLinearHeading(
//                        Pose2d(
//                            AutoConfig.BLUE_FAR_LEFT_X[1],
//                            AutoConfig.BLUE_FAR_LEFT_Y[1],
//                            0.0
//                        )
//                    )
//                    .build()
//                t2_5 = drive.trajectoryBuilder(t2.end())
//                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[2], AutoConfig.BLUE_FAR_LEFT_Y[2]))
//                    .build()
//                t3 = drive.trajectoryBuilder(t2_5.end())
//                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[3], AutoConfig.BLUE_FAR_LEFT_Y[3]))
//                    .build()
//                t4 = drive.trajectoryBuilder(t3.end())
//                    .lineToLinearHeading(
//                        Pose2d(
//                            AutoConfig.BLUE_FAR_LEFT_X[4],
//                            AutoConfig.BLUE_FAR_LEFT_Y[4],
//                            PI
//                        )
//                    )
//                    .build()
//                t5 = drive.trajectoryBuilder(t4.end())
//                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[5], AutoConfig.BLUE_FAR_LEFT_Y[5]))
//                    .build()
//                t6 = drive.trajectoryBuilder(t5.end())
//                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[6], AutoConfig.BLUE_FAR_LEFT_Y[6]))
//                    .build()
//                t7 = drive.trajectoryBuilder(t6.end())
//                    .lineTo(Vector2d(AutoConfig.BLUE_FAR_LEFT_X[7], AutoConfig.BLUE_FAR_LEFT_Y[7]))
//                    .build()
//            }
//        }
//
//        val t1follower = TrajectoryFollower(drive, t1)
//        val t2follower = TrajectoryFollower(drive, t2)
//        val t2_5follower = TrajectoryFollower(drive, t2_5)
//        val t3follower = TrajectoryFollower(drive, t3)
//        val t4follower = TrajectoryFollower(drive, t4)
//        val t5follower = TrajectoryFollower(drive, t5)
//        val t6follower = TrajectoryFollower(drive, t6)
//        val t7follower = TrajectoryFollower(drive, t7)
//        +series(
//            t1follower,
//            parallel(
//                series(
//                    delay(1.0),
//                    instant { smec.armState = ScoringMechanism.State.CLOSE_INTAKE },
//                ),
//                t2follower,
//            ),
//            instant { smec.leftClawState = ClawPositions.OPEN },
//            parallel(
//                instant { smec.armState = ScoringMechanism.State.TRAVEL },
//                // We want to follow only trajectory t3 UNLESS we are on the left path, in which case we have to follow t2_5 and t3
//                if(zone == PropZone.LEFT)
//                    series(t2_5follower, t3follower)
//                else
//                    t3follower
//            ),
//
//            // TODO: these are all of the steps for after we reach the backdrop side of the field
//            // TODO: should be more or less the same for each randomization
////            parallel(
////                instant {smec.armState = ScoringMechanism.State.DEPOSIT},
////                t4follower
////            ),
////            t5follower,
////            instant {smec.rightClawState = ClawPositions.OPEN},
////            t6follower,
////            instant {smec.armState = ScoringMechanism.State.TRAVEL},
////            t7follower
//        )

        )
    }
}