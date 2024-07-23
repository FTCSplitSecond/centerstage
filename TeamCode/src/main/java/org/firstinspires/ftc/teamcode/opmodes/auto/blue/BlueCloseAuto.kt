package org.firstinspires.ftc.teamcode.opmodes.auto.blue

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.teamcode.SplitSecondBot
import org.firstinspires.ftc.teamcode.common.component.TrajectoryFollower
import org.firstinspires.ftc.teamcode.common.ktx.adjustForAlliance
import org.firstinspires.ftc.teamcode.common.types.Alliance
import org.firstinspires.ftc.teamcode.common.types.ClawSide
import org.firstinspires.ftc.teamcode.common.types.OpModeType
import org.firstinspires.ftc.teamcode.component.claw.CloseBothClaw
import org.firstinspires.ftc.teamcode.component.claw.DropBothClaw
import org.firstinspires.ftc.teamcode.component.claw.OpenBothClaw
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem
import org.firstinspires.ftc.teamcode.subsystem.vision.detector.PropZoneDetected
import kotlin.math.PI

@Autonomous
class BlueCloseAuto: AnchorOpMode() {
    lateinit var robot: SplitSecondBot
    lateinit var deposit: DepositSubsystem
    lateinit var drive: CenterstageMecanumDrive

    val startPose = Pose2d(16.0, 62.0, PI / 2)
    val alliance = Alliance.BLUE

    override fun prerun() {
        val driver = FTCGamepad(gamepad1)

        robot = SplitSecondBot(
            hardwareMap,
            hardwareManager,
            alliance,
            OpModeType.AUTO,
            startPose,
            telemetry
        )

        deposit = robot.deposit
        drive = robot.drivetrain.drive
        robot.elbow.isEnabled = true
        robot.init(this.world)

        + OpenBothClaw(robot.claw)

        driver[Button.Key.DPAD_LEFT] onActivate instant {
            when (robot.claw.leftClaw) {
                ClawSubsystem.ClawState.OPEN -> robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.CLOSED)
                ClawSubsystem.ClawState.CLOSED -> {
                    if (deposit.armState == DepositSubsystem.State.EXTENDED_INTAKE) {
                        robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.OPEN)
                    } else if (deposit.armState == DepositSubsystem.State.CLOSED_INTAKE) {
                        robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.OPEN)
                    }
                }
                else -> robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.CLOSED)
            }
        }

        driver[Button.Key.DPAD_RIGHT] onActivate instant {
            when (robot.claw.rightClaw) {
                ClawSubsystem.ClawState.OPEN -> robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.CLOSED)
                ClawSubsystem.ClawState.CLOSED -> {
                    if (deposit.armState == DepositSubsystem.State.EXTENDED_INTAKE) {
                        robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.OPEN)
                    } else if (deposit.armState == DepositSubsystem.State.CLOSED_INTAKE) {
                        robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.OPEN)
                    }
                }
                else -> robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.CLOSED)
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
        robot.vision.stopProp()

        val startHeading = getAllianceHeading(alliance)
        val awayFromWallPosition = Vector2d(22.00, 60.0).adjustForAlliance(alliance)
        val purplePixelPoseBackdropSide = Pose2d(Vector2d(36.0, 30.0), PI).adjustForAlliance(alliance)
        val purplePixelPoseCenter = Pose2d(Vector2d(28.0, 25.5), PI).adjustForAlliance(alliance)
        val purplePixelPoseAwayFromBackdrop = Pose2d(Vector2d(12.0, 30.0), PI).adjustForAlliance(alliance)

        val purplePixelPose = when (zoneDetected) {
            PropZoneDetected.LEFT -> if (alliance == Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
            PropZoneDetected.CENTER -> purplePixelPoseCenter
            PropZoneDetected.RIGHT -> if (alliance == Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
            else -> purplePixelPoseCenter
        }

        val transitLaneY = 12.0
        val nearBackdropLaneX = 34.0
        val backdropScoreX = 42.0
        val pixelFromLaneOffset = 2.0

        val transitLanePoseAfterYellowPixel = Pose2d(
            Vector2d(-36.0, transitLaneY), PI
        ).adjustForAlliance(alliance)

        val transitLaneBackDropSide = Vector2d(nearBackdropLaneX, transitLaneY).adjustForAlliance(alliance)
        val transitLanePixelStackSide = Vector2d(-32.0, transitLaneY + pixelFromLaneOffset).adjustForAlliance(alliance)
        val stackLocation = Vector2d(-41.0, transitLaneY + pixelFromLaneOffset).adjustForAlliance(alliance)
        val backDropScoringClawOffset = 0.0 // offset to help pixels land better if needed
        val backDropZoneSpacing = 6.0
        val backDropCenterY = 36.0

        val nearBackDropCenter = Vector2d(nearBackdropLaneX, backDropCenterY + backDropScoringClawOffset).adjustForAlliance(alliance)
        val nearBackDropLeft = Vector2d(nearBackdropLaneX, nearBackDropCenter.y + backDropZoneSpacing)
        val nearBackDropRight = Vector2d(nearBackdropLaneX, nearBackDropCenter.y - backDropZoneSpacing)

        val nearBackdropPosition = when (zoneDetected) {
            PropZoneDetected.LEFT -> nearBackDropLeft
            PropZoneDetected.CENTER -> nearBackDropCenter
            PropZoneDetected.RIGHT -> nearBackDropRight
            else -> nearBackDropCenter
        }

        val backDropScoringPosition = Vector2d(backdropScoreX, nearBackdropPosition.y)  // no need to adjust for alliance (already there)

        val parkInsidePosition = Vector2d(48.0, transitLaneY).adjustForAlliance(alliance)
        val parkOutsidePosition = Vector2d(backdropScoreX, 60.0).adjustForAlliance(alliance)

        val moveAwayFromWallTrajectory = drive.trajectoryBuilder(startPose)
            .lineTo(awayFromWallPosition)
            .build()

        val moveToScorePurplePixelTrajectory = drive.trajectoryBuilder(moveAwayFromWallTrajectory.end())
            .lineToLinearHeading(purplePixelPose)
            .build()

        val moveToNearBackdropTrajectory = drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
            .lineToLinearHeading(Pose2d(nearBackdropPosition, PI))
            .build()

        val moveToNearBackdropWhiteTrajectory = drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
            .lineToLinearHeading(Pose2d(nearBackdropPosition, PI))
            .build()

        val moveToScoreBackDropTrajectory = drive.trajectoryBuilder(moveToNearBackdropTrajectory.end())
            .lineTo(backDropScoringPosition)
            .build()

        val backAwayFromBackDropTrajectory = drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
            .lineTo(nearBackdropPosition)
            .build()

        val moveToTransitLaneTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(transitLaneBackDropSide)
            .build()

        val moveToPixelStacksTrajectory = drive.trajectoryBuilder(moveToTransitLaneTrajectory.end())
            .lineTo(transitLanePixelStackSide)
            .build()

        val pickUpPixelsTrajectory = drive.trajectoryBuilder(moveToPixelStacksTrajectory.end())
            .lineTo(stackLocation)
            .build()

        val moveToTransitLaneFromPixelStacksTrajectory = drive.trajectoryBuilder(moveToPixelStacksTrajectory.end())
            .lineTo(transitLaneBackDropSide)
            .build()

        val parkInsideTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(parkInsidePosition)
            .build()

        val parkOutsideTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(parkOutsidePosition)
            .build()

        // components

        val moveAwayFromWall = TrajectoryFollower(drive, moveAwayFromWallTrajectory)
        val moveToCloseIntake = deposit.setArmState(DepositSubsystem.State.CLOSED_INTAKE)
        val moveToScorePurplePixel = TrajectoryFollower(drive, moveToScorePurplePixelTrajectory)
        val scorePurple = instant { robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.OPEN) }
        val moveToTravel = deposit.setArmState(DepositSubsystem.State.TRAVEL)
        val moveToNearBackdrop = TrajectoryFollower(drive, moveToNearBackdropTrajectory)
        val moveToNearBackdropWhite = TrajectoryFollower(drive, moveToNearBackdropWhiteTrajectory)

        val moveToDeposit = parallel(
            deposit.setPixelLevel(deposit.depositPixelLevel - 0.5),
            deposit.setArmState(DepositSubsystem.State.DEPOSIT)
        )

        val moveToScoreBackdrop = TrajectoryFollower(drive, moveToScoreBackDropTrajectory)
        val scoreBackdrop = DropBothClaw(robot.claw)

        val backAwayFromBackdrop = parallel(
            TrajectoryFollower(drive, backAwayFromBackDropTrajectory),
            series(
                delay(0.5),
                moveToTravel
            )
        )

        val moveToTransitLane = TrajectoryFollower(drive, moveToTransitLaneTrajectory)
        val moveToPixelStacks = parallel(
            TrajectoryFollower(drive, moveToPixelStacksTrajectory),
            deposit.setArmState(DepositSubsystem.State.STACK_INTAKE_CLOSED)
        )

        val pickUpPixelStacks = series(
            TrajectoryFollower(drive, pickUpPixelsTrajectory),
            CloseBothClaw(robot.claw)
        )

        val moveToTransitLaneFromPixelStacks = TrajectoryFollower(drive, moveToTransitLaneFromPixelStacksTrajectory)

        val backAwayAfterScoringWhitePixel = TrajectoryFollower(
            drive, drive.trajectoryBuilder(moveToScoreBackDropTrajectory.end())
                .forward(12.0).build()
        )

        val parkInside = TrajectoryFollower(drive, parkInsideTrajectory)
        val parkOutside = TrajectoryFollower(drive, parkOutsideTrajectory)

        val stopVisionPortal = instant { robot.vision.end(FinishReason.INTERRUPTED) }

        + series(
            moveAwayFromWall,

            parallel(
                moveToScorePurplePixel,
                series(
                    delay(0.5),
                    moveToCloseIntake
                )
            ),

            scorePurple,

            stopVisionPortal,

            parallel(
                moveToDeposit,
                moveToNearBackdrop
            ),

            delay(0.25),

            moveToScoreBackdrop,

            scoreBackdrop,

            backAwayFromBackdrop,

            moveToTravel,

            moveToTransitLane,

            moveToPixelStacks,

            pickUpPixelStacks,

            parallel(
                moveToTransitLaneFromPixelStacks,
                moveToTravel
            ),

            parallel(
                moveToNearBackdropWhite,
                series(
                    moveToDeposit,
                    deposit.setPixelLevel(4.0)
                )
            ),

            moveToScoreBackdrop,

            scoreBackdrop,

            backAwayAfterScoringWhitePixel,

            moveToTravel,

            parkInside
        )
    }
}