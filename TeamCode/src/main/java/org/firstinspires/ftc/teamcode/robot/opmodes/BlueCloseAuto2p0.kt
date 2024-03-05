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
class BlueCloseAuto2p0 : AnchorOpMode() {
    lateinit var robot: Robot
    lateinit var smec: ScoringMechanism
    lateinit var drive: CenterstageMecanumDrive
    val startPose = Pose2d(16.0, 62.0, PI / 2)
    val alliance = Alliance.BLUE

    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, startPose = startPose)
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

        // purple pixel is in the left claw, yellow is in the right
        val startHeading = getAllianceHeading(alliance)
        val awayFromWallPosition = Vector2d(22.0, 60.0).adjustForAlliance(alliance)
        val purplePixelPoseBackdropSide = Pose2d(Vector2d(36.0, 30.0), PI).adjustForAlliance(alliance)
        val purplePixelPoseCenter = Pose2d(Vector2d(28.0, 25.5), PI).adjustForAlliance(alliance)
        val purplePixelPoseAwayFromBackdrop = Pose2d(Vector2d(12.0, 30.0), PI).adjustForAlliance(alliance)
        // note here that zone right/left means different things for red and blue
        val purplePixelPose = when (zoneDetected) {
            PropZoneDetected.LEFT -> if(alliance== Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
            PropZoneDetected.CENTER, PropZoneDetected.NONE -> purplePixelPoseCenter
            PropZoneDetected.RIGHT -> if(alliance== Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
        }
        val transitLaneY = 60.0
        val nearBackDropLaneX = 34.0
        val backDropScoreX = 42.0

        val transitLanePoseAfterPurplePixel = Pose2d(Vector2d(-36.0, transitLaneY), PI).adjustForAlliance(alliance)
        val transitLaneBackDropSide = Vector2d(nearBackDropLaneX, transitLaneY).adjustForAlliance(alliance)
        val transitLanePixelStackSide = Vector2d(-48.0, transitLaneY).adjustForAlliance(alliance)
        val backDropScoringClawOffset = 0.0 // offset to help pixels land better if needed
        val backDropZoneSpacing = 6.0
        val backDropCenterY = 36.0
        val nearBackDropCenter = Vector2d(nearBackDropLaneX, backDropCenterY + backDropScoringClawOffset).adjustForAlliance(alliance)
        val nearBackDropLeft = Vector2d(nearBackDropLaneX, nearBackDropCenter.y + backDropZoneSpacing)
        val nearBackDropRight = Vector2d(nearBackDropLaneX, nearBackDropCenter.y - backDropZoneSpacing)
        val nearBackDropPosition = when (zoneDetected) {
            PropZoneDetected.LEFT ->  nearBackDropLeft
            PropZoneDetected.CENTER, PropZoneDetected.NONE -> nearBackDropCenter
            PropZoneDetected.RIGHT -> nearBackDropRight
        }
        val backDropScoringPosition = Vector2d(backDropScoreX, nearBackDropPosition.y)  // no need to adjust for alliance (already there)
        val parkInsidePosition = Vector2d(48.0, transitLaneY).adjustForAlliance(alliance)
        val parkOutsidePosition = Vector2d(backDropScoreX, 60.0).adjustForAlliance(alliance)


        // trajectories
        val moveAwayFromWallTrajectory = drive.trajectoryBuilder(startPose)
            .lineTo(awayFromWallPosition)
            .build()
        val moveToScorePurplePixelTrajectory = drive.trajectoryBuilder(moveAwayFromWallTrajectory.end())
            .lineToLinearHeading(purplePixelPose)
            .build()
//        val moveToBackDropLaneTrajectory = drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
//                .lineTo(transitLaneBackDropSide)
//                .build()
        val moveToNearBackdropTrajectory = drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
            .lineToLinearHeading(Pose2d(nearBackDropPosition, PI))
            .build()
        val moveToScoreBackDropTrajectory = drive.trajectoryBuilder(moveToNearBackdropTrajectory.end())
            .lineTo(backDropScoringPosition) // front is facing away from BB, possibly add a velocity/acceleration constraint here as we might ram the BB
            .build()
        val backAwayFromBackDropTrajectory = drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
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
        val moveToScorePurplePixel = TrajectoryFollower(drive, moveToScorePurplePixelTrajectory)
        val scorePurplePixel = instant { robot.leftClaw.position = ClawPositions.OPEN }
        val moveToTravel = smec.setArmState(ScoringMechanism.State.TRAVEL)
//        val moveToBackDropLane = TrajectoryFollower(drive, moveToBackDropLaneTrajectory)
        val moveToNearBackdrop = TrajectoryFollower(drive, moveToNearBackdropTrajectory)
        val moveToDeposit = parallel(
            smec.setDepositPixelLevel( -0.5),
            smec.setArmState(ScoringMechanism.State.DEPOSIT)
        )
        val moveToScoreBackDrop = TrajectoryFollower(drive, moveToScoreBackDropTrajectory)
        val scoreBackDrop = series(
            instant {
                DropBothClaw(robot.leftClaw, robot.rightClaw)
            }
        )
        val backAwayFromBackDrop = parallel(
            TrajectoryFollower(drive, backAwayFromBackDropTrajectory),
            series(
                delay(0.5),  // delay here is to not pull the pixel with us
                moveToTravel))

        // used for extra cycles
//        val moveToTransitLaneToPixelStacks = TrajectoryFollower(drive, moveToTransitLaneToPixelStacksTrajectory)
//        val intakePixelsFromStack = instant {  } // add pixel intake here returns to transit lane when done
//        val moveToTransitLaneFromPixelStacks = TrajectoryFollower(drive, moveToTransitLaneFromPixelStacksTrajectory)

        val parkInside = TrajectoryFollower(drive, parkInsideTrajectory)
        val parkOutside = TrajectoryFollower(drive, parkOutsideTrajectory)
        val relocalizeFromAprilTags = instant {  } // add april tag relocalization here

        // Now we schedule the commands
        +series(
            moveAwayFromWall,

            parallel(moveToScorePurplePixel, series(delay(0.5), moveToCloseIntake)),

            scorePurplePixel,

            parallel(moveToDeposit, moveToNearBackdrop,instant { robot.leftClaw.position = ClawPositions.CLOSED}), //may need delay on moveToTravel

            relocalizeFromAprilTags,

            delay(0.25),

            moveToScoreBackDrop,

            scoreBackDrop,

            backAwayFromBackDrop,

            moveToTravel,

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

            parkInside
        )

    }
}