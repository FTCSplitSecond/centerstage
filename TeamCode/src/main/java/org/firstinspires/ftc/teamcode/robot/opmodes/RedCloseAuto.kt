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
import org.firstinspires.ftc.teamcode.vision.RelocalizeFromAprilTags
import org.firstinspires.ftc.teamcode.vision.processors.PropZoneDetected
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.robot.util.AutoConfig
import org.firstinspires.ftc.teamcode.robot.util.ParkLocation
import org.firstinspires.ftc.teamcode.vision.AprilTagRelocalize
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


@Autonomous
class RedCloseAuto : AnchorOpMode() {
    lateinit var robot: Robot
    lateinit var smec: ScoringMechanism
    lateinit var drive: CenterstageMecanumDrive
    lateinit var webcam: OpenCvWebcam
    var detector = PropDetector(telemetry)
    val startPose = Pose2d(8.0, -62.0, -PI / 2)
    val alliance = Alliance.RED

    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, startPose = startPose)
        Robot.alliance = alliance
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

        //+OpenBothClaw(robot.leftClaw, robot.rightClaw)

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
        val parkLocation = AutoConfig.RED_CLOSE2P0_PARK
        val delayA = AutoConfig.RED_CLOSE2P0_DELAYS[0]
        val delayB = AutoConfig.RED_CLOSE2P0_DELAYS[1]

        val zoneDetected = detector.zone
        webcam.stopStreaming()
        //VISION SUBSYSTEM THAT ISNT WORKING
//        val zoneDetected = robot.vision.propZoneDetected
//        robot.vision.disablePropZoneDetector()

        // purple pixel is in the left claw, yellow is in the right
        val startHeading = getAllianceHeading(alliance)
        val awayFromWallPosition = Vector2d(24.0, 36.0).adjustForAlliance(alliance)
        val purplePixelPoseBackdropSide = Pose2d(Vector2d(33.0, 30.0), PI).adjustForAlliance(alliance)
        val purplePixelPoseCenter = Pose2d(Vector2d(26.0, 23.0), PI).adjustForAlliance(alliance)
        val purplePixelPoseAwayFromBackdrop = Pose2d(Vector2d(11.5, 32.0), PI).adjustForAlliance(alliance)
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
        val backDropZoneSpacing = 6.5
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

        val parkInsidePosition = Vector2d(42.0, 16.0).adjustForAlliance(alliance)
        val parkCenterPosition = Vector2d(42.0, 44.0).adjustForAlliance(alliance)
        val parkOutsidePosition = Vector2d(48.0, 60.0).adjustForAlliance(alliance)
        val parkPosition = when(parkLocation) {
            ParkLocation.INSIDE -> parkInsidePosition
            ParkLocation.CENTER -> parkCenterPosition
            ParkLocation.OUTSIDE -> parkOutsidePosition
        }

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
        val moveAwayFromPurplePixelTrajectory = drive.trajectoryBuilder(moveToScorePurplePixelTrajectory.end())
            .lineTo(Vector2d(moveToScorePurplePixelTrajectory.end().x + 4.0, moveToScorePurplePixelTrajectory.end().y)) //Moves backward first to avoid pushing purple pixel
            .build()
        val moveToNearBackdropTrajectory = drive.trajectoryBuilder(moveAwayFromPurplePixelTrajectory.end())
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

        val parkTrajectory = drive.trajectoryBuilder(backAwayFromBackDropTrajectory.end())
            .lineTo(parkPosition)
            .build()

        // commands
        val moveAwayFromWall = TrajectoryFollower(drive, moveAwayFromWallTrajectory)
        val moveToCloseIntake = smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
        val moveToScorePurplePixel = TrajectoryFollower(drive, moveToScorePurplePixelTrajectory)
        val scorePurplePixel = instant { robot.leftClaw.position= ClawPositions.OPEN }
        val moveToTravel = smec.setArmState(ScoringMechanism.State.TRAVEL)
//        val moveToBackDropLane = TrajectoryFollower(drive, moveToBackDropLaneTrajectory)
        val moveToNearBackdrop = series( TrajectoryFollower(drive, moveAwayFromPurplePixelTrajectory), TrajectoryFollower(drive, moveToNearBackdropTrajectory) )
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

        // used for extra cycles
//        val moveToTransitLaneToPixelStacks = TrajectoryFollower(drive, moveToTransitLaneToPixelStacksTrajectory)
//        val intakePixelsFromStack = instant {  } // add pixel intake here returns to transit lane when done
//        val moveToTransitLaneFromPixelStacks = TrajectoryFollower(drive, moveToTransitLaneFromPixelStacksTrajectory)

        val park = TrajectoryFollower(drive, parkTrajectory)
        val relocalizeFromAprilTags = AprilTagRelocalize(robot.vision, robot)

        // Now we schedule the commands
        +series(

            delay(delayA),

            moveAwayFromWall,

            moveToCloseIntake,

            moveToScorePurplePixel,

            scorePurplePixel,

            delay(delayB),

            parallel(
                moveToDeposit,
                moveToNearBackdrop,
                series( delay(0.5), instant { robot.rightClaw.position = ClawPositions.CLOSED } )
            ),

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

            park
        )
    }
}