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
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.mecanum.commands.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.robot.util.Alliance
import org.firstinspires.ftc.teamcode.robot.util.AutoConfig
import org.firstinspires.ftc.teamcode.robot.util.ParkLocation
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.robot.util.adjustForAlliance
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.vision.AprilTagRelocalize


@Autonomous
class BlueCloseAuto2p1 : AnchorOpMode() {
    lateinit var robot: Robot
    lateinit var smec: ScoringMechanism
    lateinit var drive: CenterstageMecanumDrive
    lateinit var webcam: OpenCvWebcam
    var detector = PropDetector(telemetry)
    val startPose = Pose2d(-33.0, 62.0, PI / 2)
    val alliance = Alliance.BLUE

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
        parkLocation = AutoConfig.BLUE_CLOSE2P1_PARK
        delayA = AutoConfig.BLUE_CLOSE2P1_DELAYS[0]

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
        //val delayA = AutoConfig.BLUE_CENTER_DELAYS[0]
        val delayB = AutoConfig.BLUE_CLOSE2P1_DELAYS[1]
        val delayC = AutoConfig.BLUE_CLOSE2P1_DELAYS[2]

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
        val awayFromWallPosition = Vector2d(24.0, 36.0).adjustForAlliance(alliance)
        val purplePixelPoseBackdropSide = Pose2d(Vector2d(34.0, 30.0), PI).adjustForAlliance(alliance)
        val purplePixelPoseCenter = Pose2d(Vector2d(26.0, 23.0), PI).adjustForAlliance(alliance)
        val purplePixelPoseAwayFromBackdrop = Pose2d(Vector2d(11.5, 32.0), PI).adjustForAlliance(alliance)
        // note here that zone right/left means different things for red and blue
        val purplePixelPose = when (zoneDetected) {
            PropZoneDetected.LEFT -> if(alliance== Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
            PropZoneDetected.CENTER, PropZoneDetected.NONE -> purplePixelPoseCenter
            PropZoneDetected.RIGHT -> if(alliance== Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
        }

        val transitLaneY = 58.0
        val nearBackDropLaneX = 36.0
        val backDropScoreX = 43.5


        val transitLanePoseAfterPurplePixel = Pose2d(Vector2d(20.0, transitLaneY), PI + spinOffset).adjustForAlliance(alliance)

        val transitLaneBackDropSide = Vector2d(nearBackDropLaneX, transitLaneY).adjustForAlliance(alliance)
        val transitLanePixelStackSide = Vector2d(-55.0, transitLaneY).adjustForAlliance(alliance)

        val rightClawStackPose = Pose2d(Vector2d(-56.5, 29.0), PI).adjustForAlliance(alliance)

        val backDropScoringClawOffset = -4.0 // offset to help pixels land better if needed
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
            .lineTo(awayFromWallPosition)
            .build()
        val moveToScorePurplePixelTrajectory = drive.trajectoryBuilder(moveAwayFromWallTrajectory.end())
            .lineToLinearHeading(purplePixelPose)
            .build()
        val moveToCloseTransitLaneTrajectory = drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
            .lineToLinearHeading(transitLanePoseAfterPurplePixel)
            .build()
        val moveToPixelStacksTrajectory = drive.trajectoryBuilder(moveToCloseTransitLaneTrajectory.end())
            .lineTo(transitLanePixelStackSide)
            .build()
        val moveToLeftClawStackTrajectory = drive.trajectoryBuilder(moveToPixelStacksTrajectory.end())
            .lineToLinearHeading(Pose2d(Vector2d(rightClawStackPose.x + 1.0, rightClawStackPose.y), rightClawStackPose.heading))
            .build()
        val moveToLeftClawStackIntakeTrajectory = drive.trajectoryBuilder(moveToLeftClawStackTrajectory.end())
            .lineToLinearHeading(rightClawStackPose)
            .build()
        val moveToTransitLaneFromPixelStacksTrajectory = drive.trajectoryBuilder(moveToLeftClawStackIntakeTrajectory.end())
            .lineTo(transitLanePixelStackSide)
            .build()
        val moveToBackDropLaneTrajectory = drive.trajectoryBuilder(moveToTransitLaneFromPixelStacksTrajectory.end())
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
        val parkTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(parkPosition)
            .build()
        val afterParkTrajectory = drive.trajectoryBuilder(parkTrajectory.end())
            .lineTo(Vector2d(parkPosition.x + afterParkOffsetX, parkPosition.y))
            .build()

        // commands
        val moveAwayFromWall = TrajectoryFollower(drive, moveAwayFromWallTrajectory)
        val goToCloseIntake = smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
        val moveToScorePurplePixel = TrajectoryFollower(drive, moveToScorePurplePixelTrajectory)
        val scorePurplePixel = instant { robot.rightClaw.position = ClawPositions.OPEN }
        val moveToTravel = smec.setArmState(ScoringMechanism.State.TRAVEL)
        val moveToCloseTransitLane = TrajectoryFollower(drive, moveToCloseTransitLaneTrajectory)

        //Pixel Stacks
        val moveToPixelStacksLane = TrajectoryFollower(drive, moveToPixelStacksTrajectory)
        val moveToLeftClawStackIntake = TrajectoryFollower(drive, moveToLeftClawStackTrajectory)
        val moveLeftClawCloserTrajectory = TrajectoryFollower(drive, moveToLeftClawStackIntakeTrajectory)
        val moveToTransitLaneFromPixelStacks = TrajectoryFollower(drive, moveToTransitLaneFromPixelStacksTrajectory)

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

        val park = series( TrajectoryFollower(drive, parkTrajectory), TrajectoryFollower(drive, afterParkTrajectory) )
        val relocalizeFromAprilTags = AprilTagRelocalize(robot.vision, robot)

        // Now we schedule the commands
        +series(

            delay(delayA),

            moveAwayFromWall,

            goToCloseIntake,

            moveToScorePurplePixel,

            scorePurplePixel,

            moveToTravel,

            delay(delayB),

            moveToCloseTransitLane,

            //delay(8.0),

            moveToPixelStacksLane,

            moveToLeftClawStackIntake,

            series(
                parallel(
                    instant { robot.rightClaw.position = ClawPositions.OPEN },
                    smec.setArmState(ScoringMechanism.State.STACK_INTAKE_CLOSE)
                ),
                moveLeftClawCloserTrajectory,
                instant { robot.rightClaw.position = ClawPositions.CLOSED },
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            ),

            moveToTransitLaneFromPixelStacks,

            moveToBackDropLane,

            delay(delayC),

            parallel(moveToNearBackdrop, moveToDeposit),

            relocalizeFromAprilTags,

            moveToScoreBackDrop,

            scoreBackDrop,

            backAwayFromBackDrop,

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