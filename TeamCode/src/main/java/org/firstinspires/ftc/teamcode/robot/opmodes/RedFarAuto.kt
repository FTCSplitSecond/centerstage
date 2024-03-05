package org.firstinspires.ftc.teamcode.robot.opmodes

import PropDetector
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
import org.firstinspires.ftc.teamcode.robot.util.adjustForAlliance
import org.firstinspires.ftc.teamcode.vision.processors.PropZoneDetected
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.PI


@Autonomous
class RedFarAuto : AnchorOpMode() {
    lateinit var smec: ScoringMechanism
    lateinit var drive: CenterstageMecanumDrive
    val startPose = Pose2d(-40.0, -62.0, PI / 2)
    val alliance = Alliance.RED
    val robot = Robot(hardwareMap, this.hardwareManager, telemetry, startPose = startPose)

    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        Robot.alliance = alliance
        smec = robot.scoringMechanism
        drive = robot.driveBase.dt
        robot.elbow.isEnabled = true
        robot.init(this.world)

        +OpenBothClaw(robot.leftClaw, robot.rightClaw)

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
        val zoneDetected = robot.vision.propZoneDetected
        robot.vision.disablePropZoneDetector()

        val spinOffset = when (alliance) {
            Alliance.RED -> 0.0001
            Alliance.BLUE -> -0.0001
        }
//        val startHeading = getAllianceHeading(alliance)
//        val startPose = Pose2d(-32.0, 62.0, startHeading).adjustForAlliance(alliance)
        val awayFromWallPosition = Vector2d(-45.0, 60.0).adjustForAlliance(alliance)

        val purplePixelPoseBackdropSide =
            Pose2d(Vector2d(-35.0, 30.0), 0.0).adjustForAlliance(alliance)
        val purplePixelPoseCenter =
            Pose2d(Vector2d(-36.0, 13.5), startPose.heading).adjustForAlliance(alliance)
        val purplePixelPoseAwayFromBackdrop =
            Pose2d(Vector2d(-44.0, 17.0), startPose.heading).adjustForAlliance(alliance)
        val purplePixelPose = when (zoneDetected) {
            PropZoneDetected.LEFT -> if (alliance == Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
            PropZoneDetected.CENTER, PropZoneDetected.NONE -> purplePixelPoseCenter
            PropZoneDetected.RIGHT -> if (alliance == Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
        }

        val stackAxis = Pose2d(Vector2d(-36.0, 17.0), PI).adjustForAlliance(alliance)
        val stackLocation = Vector2d(-57.5, 17.0).adjustForAlliance(alliance)

        val transitLaneY = 12.0
        val nearBackDropLaneX = 34.0
        val backDropScoreX = 42.0

        val transitLanePoseAfterPurplePixel =
            Pose2d(Vector2d(-36.0, transitLaneY), PI + spinOffset).adjustForAlliance(alliance)
        val transitLaneBackDropSide =
            Vector2d(nearBackDropLaneX, transitLaneY).adjustForAlliance(alliance)
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
            Vector2d(nearBackDropLaneX, nearBackDropCenter.y + backDropZoneSpacing)
        val nearBackDropRight =
//            Vector2d(nearBackDropLaneX, nearBackDropCenter.y - backDropZoneSpacing)
            Vector2d(nearBackDropLaneX, 30.0)
        val nearBackDropPosition = when (zoneDetected) {
            PropZoneDetected.LEFT -> nearBackDropLeft
            PropZoneDetected.CENTER, PropZoneDetected.NONE -> nearBackDropCenter
            PropZoneDetected.RIGHT -> nearBackDropRight
        }
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
        // UNUSED W/ WHITE PIXEL
        val moveToFarTransitLaneTrajectory =
            drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
                .lineToLinearHeading(transitLanePoseAfterPurplePixel)
                .build()
        val moveToBackDropLaneTrajectory =
            drive.trajectoryBuilder(moveToStackTrajectory.end())
                .lineTo(transitLaneBackDropSide)
                .build()
        val moveToNearBackdropTrajectory =
            drive.trajectoryBuilder(moveToBackDropLaneTrajectory.end())
                .lineToLinearHeading(Pose2d(nearBackDropPosition, PI))
                .build()
        val moveToScoreBackDropTrajectory =
            drive.trajectoryBuilder(moveToNearBackdropTrajectory.end())
                .lineTo(backDropScoringPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
                .build()
        val backAwayFromBackDropTrajectory =
            drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
                .lineTo(nearBackDropPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
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
        val moveToCloseIntake = smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
        val alignWithStack = TrajectoryFollower(drive, moveToAlignWithStackTrajectory)
        val moveToStack = TrajectoryFollower(drive, moveToStackTrajectory)
        val moveToScorePurplePixel = TrajectoryFollower(drive, moveToScorePurplePixelTrajectory)
        val scorePurplePixel = instant { robot.leftClaw.position = ClawPositions.OPEN }
        val moveToTravel = smec.setArmState(ScoringMechanism.State.TRAVEL)
        val moveToFarTransitLane = TrajectoryFollower(drive, moveToFarTransitLaneTrajectory)
        val moveToBackDropLane = TrajectoryFollower(drive, moveToBackDropLaneTrajectory)
        val moveToNearBackdrop = TrajectoryFollower(drive, moveToNearBackdropTrajectory)
        val moveToDeposit = parallel(
            smec.setDepositPixelLevel(-0.5),
            smec.setArmState(ScoringMechanism.State.DEPOSIT)
        )
        val moveToScoreBackDrop = TrajectoryFollower(drive, moveToScoreBackDropTrajectory)
        val scoreBackDrop =DropBothClaw(robot.leftClaw, robot.rightClaw)
        val backAwayFromBackDrop = parallel(
            TrajectoryFollower(drive, backAwayFromBackDropTrajectory),
            series(
                delay(0.5),  // delay here is to not pull the pixel with us
                moveToTravel
            )
        )

        // used for extra cycles
//        val moveToTransitLaneToPixelStacks = TrajectoryFollower(drive, moveToTransitLaneToPixelStacksTrajectory)
//        val intakePixelsFromStack = instant {  } // add pixel intake here returns to transit lane when done
//        val moveToTransitLaneFromPixelStacks = TrajectoryFollower(drive, moveToTransitLaneFromPixelStacksTrajectory)

        val parkInside = TrajectoryFollower(drive, parkInsideTrajectory)
        val parkOutside = TrajectoryFollower(drive, parkOutsideTrajectory)
        val relocalizeFromAprilTags = instant { } // add april tag relocalization here

        // Now we schedule the commands
        +series(
            moveAwayFromWall,

            parallel(moveToScorePurplePixel, moveToCloseIntake),


            delay(0.5),

            scorePurplePixel,

            delay(10.0),

            parallel(
                smec.setArmState(ScoringMechanism.State.STACK_INTAKE),
                series(
                    delay(0.5),
                    alignWithStack,
                )
            ),

            moveToStack,

            instant {robot.leftClaw.position = ClawPositions.CLOSED},

//            delay(10.0),

            parallel(
                moveToTravel,

                parallel(
                    series(
                        delay(1.0),
                        moveToDeposit
                    ),
                    moveToBackDropLane
                )
//                instant { smec.leftClawState = ClawPositions.CLOSED}
            ), //may need delay on moveToTravel

//            moveToBackDropLane, // here we are at transitLaneBackDropSide

//            parallel(moveToNearBackdrop, moveToDeposit),

            moveToDeposit,
            smec.setDepositPixelLevel(0.0),

            moveToNearBackdrop,

            relocalizeFromAprilTags,

            moveToScoreBackDrop,

            scoreBackDrop,

            backAwayFromBackDrop,

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

            parallel(moveToTravel, parkOutside)
        )
    }
}