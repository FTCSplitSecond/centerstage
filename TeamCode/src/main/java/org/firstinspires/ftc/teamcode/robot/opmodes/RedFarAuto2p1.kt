package org.firstinspires.ftc.teamcode.robot.opmodes

import PropDetector
import org.firstinspires.ftc.teamcode.vision.processors.PropZoneDetected
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
import org.firstinspires.ftc.teamcode.claw.commands.DropBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.mecanum.commands.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.robot.util.Alliance
import org.firstinspires.ftc.teamcode.robot.util.AutoConfig
import org.firstinspires.ftc.teamcode.robot.util.ParkLocation
import org.firstinspires.ftc.teamcode.robot.util.adjustForAlliance
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.vision.AprilTagRelocalize
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.PI


@Autonomous
class RedFarAuto2p1 : AnchorOpMode() {
    lateinit var robot: Robot
    lateinit var smec: ScoringMechanism
    lateinit var drive: CenterstageMecanumDrive
    lateinit var webcam: OpenCvWebcam
    var detector = PropDetector(telemetry)
    val startPose = Pose2d(-40.0, -62.0, -PI / 2)
    val alliance = Alliance.RED

    lateinit var parkLocation: ParkLocation
    var delayA: Double = 0.0
    
    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        Robot.alliance = alliance
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, startPose = startPose)
        smec = robot.scoringMechanism
        drive = robot.driveBase.dt
        robot.elbow.isEnabled = true
        robot.init(this.world)
        parkLocation = AutoConfig.RED_FAR2P1_PARK
        delayA = AutoConfig.RED_FAR2P1_DELAYS[0]

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
        driver[Button.Key.SQUARE] onActivate instant {
            parkLocation = when(parkLocation) {
                ParkLocation.INSIDE -> ParkLocation.CENTER
                ParkLocation.CENTER -> ParkLocation.OUTSIDE
                ParkLocation.OUTSIDE -> ParkLocation.INSIDE
            }
        }
        driver[Button.Key.CIRCLE] onActivate instant {
            parkLocation = when(parkLocation) {
                ParkLocation.INSIDE -> ParkLocation.OUTSIDE
                ParkLocation.CENTER -> ParkLocation.INSIDE
                ParkLocation.OUTSIDE -> ParkLocation.CENTER
            }
        }
        telemetry.addLine(when(parkLocation) {
            ParkLocation.INSIDE -> "Park Inside"
            ParkLocation.CENTER -> "Park Center"
            ParkLocation.OUTSIDE -> "Park Outside"
        })
        driver[Button.Key.TRIANGLE] onActivate instant {
            delayA = (delayA + 1.0).coerceIn(0.0, 30.0)
        }
        driver[Button.Key.CROSS] onActivate instant {
            delayA = (delayA - 1.0).coerceIn(0.0, 30.0)
        }
        telemetry.addLine("Start Pose Delay: $delayA")
    }
    fun getAllianceHeading(alliance: Alliance): Double {
        return when (alliance) {
            Alliance.RED -> -PI / 2
            Alliance.BLUE -> PI / 2
        }
    }

    override fun run() {
        //val parkLocation = AutoConfig.RED_FAR_PARK
        //val delayA = AutoConfig.RED_FAR_DELAYS[0]
        val delayB = AutoConfig.RED_FAR2P1_DELAYS[1]
        val delayC = AutoConfig.RED_FAR2P1_DELAYS[2]

        val zoneDetected = detector.zone
        webcam.stopStreaming()
        //VISION SUBSYSTEM THAT ISNT WORKING
//        val zoneDetected = robot.vision.propZoneDetected
//        robot.vision.disablePropZoneDetector()

        val spinOffset = when(alliance) {
            Alliance.RED -> 0.0001
            Alliance.BLUE -> -0.0001
        }
//        val startHeading = getAllianceHeading(alliance)
//        val startPose = Pose2d(-32.0, 62.0, startHeading).adjustForAlliance(alliance)
        val awayFromWallPosition = Pose2d(Vector2d(-36.0, 36.0), startPose.heading).adjustForAlliance(alliance)

        val purplePixelPoseBackdropSide = Pose2d(Vector2d(-55.0, 30.0), PI).adjustForAlliance(alliance)
        val purplePixelPoseCenter = Pose2d(Vector2d(-36.0, 14.0), startPose.heading).adjustForAlliance(alliance)
        val purplePixelPoseAwayFromBackdrop = Pose2d(Vector2d(-48.5, 17.0), startPose.heading).adjustForAlliance(alliance)
        val purplePixelPose = when (zoneDetected) {
            PropZoneDetected.LEFT -> if(alliance== Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
            PropZoneDetected.CENTER, PropZoneDetected.NONE -> purplePixelPoseCenter
            PropZoneDetected.RIGHT -> if(alliance== Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
        }



        val transitLaneY = 12.0
        val nearBackDropLaneX = 38.0
        val backDropScoreX = 46.0

        val transitLanePoseAfterPurplePixel = Pose2d(Vector2d(-50.0, 9.5), PI + spinOffset).adjustForAlliance(alliance)

        val poseAfterPurplePixel = when (zoneDetected) {
            PropZoneDetected.LEFT -> Pose2d(Vector2d(transitLanePoseAfterPurplePixel.x + 0.01, transitLanePoseAfterPurplePixel.y), startPose.heading)
            else -> Pose2d(Vector2d(transitLanePoseAfterPurplePixel.x + 0.01, transitLanePoseAfterPurplePixel.y), transitLanePoseAfterPurplePixel.heading)
        }

        val transitLaneBackDropSide = Vector2d(nearBackDropLaneX, transitLaneY).adjustForAlliance(alliance)

        val leftClawStackPose = Pose2d(Vector2d(-58.0, 10.0), PI).adjustForAlliance(alliance)

        val backDropScoringClawOffset = 0.0 //  offset to help pixels land better if needed
        val backDropZoneSpacing = 7.0
        val backDropCenterY = 36.0
        val nearBackDropCenter = Vector2d(nearBackDropLaneX, backDropCenterY + backDropScoringClawOffset).adjustForAlliance(alliance)
        val nearBackDropLeft = Vector2d(nearBackDropLaneX, nearBackDropCenter.y + backDropZoneSpacing)
        val nearBackDropRight = Vector2d(nearBackDropLaneX, nearBackDropCenter.y - backDropZoneSpacing)
        val nearBackDropPosition = when (zoneDetected) {
            PropZoneDetected.LEFT -> nearBackDropLeft
            PropZoneDetected.CENTER, PropZoneDetected.NONE -> nearBackDropCenter
            PropZoneDetected.RIGHT -> nearBackDropRight
        }
        val backDropScoringPosition = Vector2d(backDropScoreX, nearBackDropPosition.y)  // no need to adjust for alliance (already there)

        val parkInsidePosition = Vector2d(42.0, 16.0).adjustForAlliance(alliance)
        val parkCenterPosition = Vector2d(42.0, 44.0).adjustForAlliance(alliance)
        val parkOutsidePosition = Vector2d(48.0, 60.0).adjustForAlliance(alliance)
        val parkPosition = when(parkLocation) {
            ParkLocation.INSIDE -> parkInsidePosition
            ParkLocation.CENTER -> parkCenterPosition
            ParkLocation.OUTSIDE -> parkOutsidePosition
        }

        val afterParkOffsetX = when(parkLocation) {
            ParkLocation.INSIDE -> 0.001
            ParkLocation.CENTER -> 0.001
            ParkLocation.OUTSIDE -> 6.0
        }

        // trajectories
        val moveAwayFromWallTrajectory = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(awayFromWallPosition)
            .build()
        val moveToScorePurplePixelTrajectory = drive.trajectoryBuilder(moveAwayFromWallTrajectory.end())
                .lineToLinearHeading(purplePixelPose)
                .build()
        val moveToAfterPurplePixelTrajectory = drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
            .lineToLinearHeading(poseAfterPurplePixel)
            .build()
        val moveToFarTransitLaneTrajectory = drive.trajectoryBuilder(moveToAfterPurplePixelTrajectory.end())
                .lineToLinearHeading(transitLanePoseAfterPurplePixel)
                .build()
        val moveToLeftClawStackTrajectory = drive.trajectoryBuilder(moveToFarTransitLaneTrajectory.end())
                .lineToLinearHeading(leftClawStackPose)
                .build()
        val moveToTransitLaneFromPixelStacks = drive.trajectoryBuilder(moveToLeftClawStackTrajectory.end())
                .lineToLinearHeading(transitLanePoseAfterPurplePixel)
                .build()
        val moveToBackDropLaneTrajectory = drive.trajectoryBuilder(moveToTransitLaneFromPixelStacks.end())
                .lineTo(transitLaneBackDropSide)
                .build()
        val moveToNearBackdropTrajectory = drive.trajectoryBuilder(moveToBackDropLaneTrajectory.end())
                .lineToLinearHeading(Pose2d(nearBackDropPosition, PI))
                .build()
        val moveToScoreBackDropTrajectory = drive.trajectoryBuilder(moveToNearBackdropTrajectory.end())
                .lineTo(backDropScoringPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
                .build()
        val backAwayFromBackDropTrajectory = drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
                .lineTo(nearBackDropPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
                .build()
        //FOR CENTER STACKS
//        val moveToTransitLaneFromDepositTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
//                .lineTo(transitLaneBackDropSide)
//                .build()
//        val moveToNearCenterStackTrajectory = drive.trajectoryBuilder(moveToTransitLaneFromDepositTrajectory.end())
//                .lineTo(Vector2d(nearCenterStackPose.x + 0.1, nearCenterStackPose.y + 0.1))
//                .build()
//        val turnToFaceCenterStackTrajectory = drive.trajectoryBuilder(moveToNearCenterStackTrajectory.end())
//                .lineToLinearHeading(Pose2d(nearCenterStackPose, centerStackAngle))
//                .build()
//        val moveToCenterStackLeftClawTrajectory = drive.trajectoryBuilder(turnToFaceCenterStackTrajectory.end())
//                .lineToLinearHeading(centerStackLeftClawPose)
//                .build()
//        val moveToCenterStackRightClawTrajectory = drive.trajectoryBuilder(moveToCenterStackLeftClawTrajectory.end())
//                .lineToLinearHeading(centerStackRightClawPose)
//                .build()
//        val moveToTransitLaneFromCenterStackTrajectory = drive.trajectoryBuilder(moveToCenterStackRightClawTrajectory.end())
//                .lineToLinearHeading(Pose2d(nearCenterStackPose, PI))
//                .build()
//        val moveToBackdropLaneFromCenterTrajectory = drive.trajectoryBuilder(moveToTransitLaneFromCenterStackTrajectory.end())
//                .lineTo(transitLaneBackDropSide)
//                .build()
//        val moveToNearBackdropAfterCenterTrajectory = drive.trajectoryBuilder(moveToBackdropLaneFromCenterTrajectory.end())
//            .lineToLinearHeading(Pose2d(nearBackDropPosition, PI))
//            .build()
//        val moveToScoreBackDropAfterCenterTrajectory = drive.trajectoryBuilder(moveToNearBackdropAfterCenterTrajectory.end())
//            .lineTo(backDropScoringPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
//            .build()
//        val backAwayFromBackDropAfterCenterTrajectory = drive.trajectoryBuilder(moveToScoreBackDropAfterCenterTrajectory.end())
//            .lineTo(nearBackDropPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
//            .build()
//        val parkCenterTrajectory = drive.trajectoryBuilder(backAwayFromBackDropAfterCenterTrajectory.end())
//                .lineTo(parkCenterPosition)
//                .build()
        val parkTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(parkPosition)
            .build()
        val afterParkTrajectory = drive.trajectoryBuilder(parkTrajectory.end())
            .lineTo(Vector2d(parkPosition.x + afterParkOffsetX, parkPosition.y))
            .build()


        // commands
        val moveAwayFromWall = TrajectoryFollower(drive, moveAwayFromWallTrajectory)
        val setArmStateForPurple = when(zoneDetected) {
            PropZoneDetected.RIGHT -> series(
                smec.setArmState(ScoringMechanism.State.PURPLE_DROP),
                SetTelescopePosition(robot.telescope, TelescopePosition.PurplePush)
            )
            else -> smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
        }
        val moveToScorePurplePixel = TrajectoryFollower(drive, moveToScorePurplePixelTrajectory)
        val scorePurplePixel = instant { robot.leftClaw.position = ClawPositions.OPEN }
        val moveToTravel = smec.setArmState(ScoringMechanism.State.TRAVEL)
        val moveToFarTransitLane = series(
            TrajectoryFollower(drive, moveToAfterPurplePixelTrajectory),
            TrajectoryFollower(drive, moveToFarTransitLaneTrajectory)
        )
        val moveToWallStackIntake = TrajectoryFollower(drive, moveToLeftClawStackTrajectory)
        val moveToTransitLaneFromWallStack = TrajectoryFollower(drive, moveToTransitLaneFromPixelStacks)
        val moveToBackDropLane = TrajectoryFollower(drive, moveToBackDropLaneTrajectory)
        val moveToNearBackdrop = TrajectoryFollower(drive, moveToNearBackdropTrajectory)
        val moveToDeposit = parallel(
            smec.setDepositPixelLevel(-0.5),
            smec.setArmState(ScoringMechanism.State.DEPOSIT)
        )
        val moveToScoreBackDrop = TrajectoryFollower(drive, moveToScoreBackDropTrajectory)
        val scoreBackDrop = DropBothClaw(robot.leftClaw, robot.rightClaw)
        val backAwayFromBackDrop = parallel(
            TrajectoryFollower(drive, backAwayFromBackDropTrajectory),
            series(
                delay(0.5),  // delay here is to not pull the pixel with us
                moveToTravel))

        // FOR CENTER STACKS
//        val moveToTransitLaneFromDeposit = TrajectoryFollower(drive, moveToTransitLaneFromDepositTrajectory)
//        val moveToNearCenterStack = TrajectoryFollower(drive, moveToNearCenterStackTrajectory)
//        val turnToFaceCenterStack = TrajectoryFollower(drive, turnToFaceCenterStackTrajectory)
//        val moveToCenterStackLeftClaw = TrajectoryFollower(drive, moveToCenterStackLeftClawTrajectory)
//        val moveToCenterStackRightClaw = TrajectoryFollower(drive, moveToCenterStackRightClawTrajectory)
//        val moveToTransitLaneFromCenterStack = TrajectoryFollower(drive, moveToTransitLaneFromCenterStackTrajectory)
//        val moveToBackdropLaneFromCenter = TrajectoryFollower(drive, moveToBackdropLaneFromCenterTrajectory)
//        val moveToNearBackdropAfterCenter = TrajectoryFollower(drive, moveToNearBackdropAfterCenterTrajectory)
//        val moveToScoreBackdropAfterCenter = TrajectoryFollower(drive, moveToScoreBackDropAfterCenterTrajectory)
//        val backAwayFromBackdropAfterCenter = TrajectoryFollower(drive, backAwayFromBackDropAfterCenterTrajectory)

        val park = series( TrajectoryFollower(drive, parkTrajectory), TrajectoryFollower(drive, afterParkTrajectory) )
        val relocalizeFromAprilTags = AprilTagRelocalize(robot.vision, robot)

        // Now we schedule the commands
        +series(

            delay(delayA),

            moveAwayFromWall,

            moveToScorePurplePixel,

            setArmStateForPurple,

            instant { robot.leftClaw.position = ClawPositions.OPEN },

            moveToTravel,

            delay(delayB),

            moveToFarTransitLane,

            series(
                parallel(
                    instant { robot.leftClaw.position = ClawPositions.OPEN },
                    smec.setArmState(ScoringMechanism.State.STACK_INTAKE_CLOSE)
                ),
                moveToWallStackIntake,
                instant { robot.leftClaw.position = ClawPositions.CLOSED },
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            ),

            moveToTransitLaneFromWallStack,

            moveToBackDropLane,

            delay(delayC),

            parallel(moveToNearBackdrop, moveToDeposit),

            relocalizeFromAprilTags,

            moveToScoreBackDrop,

            scoreBackDrop,

            backAwayFromBackDrop,

            // FOR CENTER STACKS
//            moveToTransitLaneFromDeposit,
//
//            moveToNearCenterStack,
//
//            turnToFaceCenterStack,
//
//            moveToCenterStackLeftClaw,
//
//            intakeStackLeftClaw,
//
//            moveToCenterStackRightClaw,
//
//            intakeStackRightClaw,
//
//            moveToTransitLaneFromCenterStack,
//
//            moveToBackdropLaneFromCenter,
//
//            parallel(moveToNearBackdropAfterCenter, moveToDeposit),
//
//            relocalizeFromAprilTags,
//
//            moveToScoreBackdropAfterCenter,
//
//            scoreBackDrop,
//
//            backAwayFromBackdropAfterCenter,

            parallel(moveToTravel, park)
        )

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


    }
}