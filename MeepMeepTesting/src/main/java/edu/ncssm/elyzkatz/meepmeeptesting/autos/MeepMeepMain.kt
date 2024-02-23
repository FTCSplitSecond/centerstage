package edu.ncssm.elyzkatz.meepmeeptesting.autos

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import kotlin.math.PI

fun main() {
    val meepMeep = MeepMeep(800)

    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 50.0, Math.toRadians(360.0), Math.toRadians(180.0), 13.0)
            .setDimensions(15.5, 15.75)
            .followTrajectorySequence { drive ->
                farAuto(
                    drive,
                    Alliance.RED,
                    PropZone.RIGHT
                )
            }

    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}

fun getAllianceHeading(alliance: Alliance): Double {
    return when (alliance) {
        Alliance.RED -> -PI / 2
        Alliance.BLUE -> PI / 2
    }
}

fun nearAuto(drive: DriveShim, alliance: Alliance, zone: PropZone): TrajectorySequence {
    // purple pixel is in the left claw, yellow is in the right
    val startHeading = getAllianceHeading(alliance)
    val startPose = Pose2d(8.0, 62.0, startHeading).adjustForAlliance(alliance)
    val awayFromWallPosition = Vector2d(16.0, 60.0).adjustForAlliance(alliance)
    val purplePixelPoseBackdropSide = Pose2d(Vector2d(36.0, 30.0), PI).adjustForAlliance(alliance)
    val purplePixelPoseCenter = Pose2d(Vector2d(28.0, 25.5), PI).adjustForAlliance(alliance)
    val purplePixelPoseAwayFromBackdrop = Pose2d(Vector2d(12.0, 30.0), PI).adjustForAlliance(alliance)
    // note here that zone right/left means different things for red and blue
    val purplePixelPose = when (zone) {
        PropZone.LEFT -> if(alliance== Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
        PropZone.CENTER, PropZone.UNKNOWN -> purplePixelPoseCenter
        PropZone.RIGHT -> if(alliance== Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
    }
    val transitLaneY = 60.0
    val nearBackDropLaneX = 37.0
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
    val nearBackDropPosition = when (zone) {
        PropZone.LEFT ->  nearBackDropLeft
        PropZone.CENTER, PropZone.UNKNOWN -> nearBackDropCenter
        PropZone.RIGHT -> nearBackDropRight
    }
    val backDropScoringPosition = Vector2d(backDropScoreX, nearBackDropPosition.y)  // no need to adjust for alliance (already there)
    val parkPosition = Vector2d(48.0, transitLaneY).adjustForAlliance(alliance)



    val sb = drive.trajectorySequenceBuilder(startPose)
    with(sb) {
        // move away from wall
        lineTo(awayFromWallPosition)
        // move to purple pixel
        lineToLinearHeading(purplePixelPose)
        waitSeconds(2.0)

        // move to near back drop
        lineToLinearHeading(Pose2d(nearBackDropPosition, PI))

        // move to score back drop
        lineTo(backDropScoringPosition)
        lineTo(nearBackDropPosition)

        // move to transit lane to pixel stacks
//        lineTo(transitLaneBackDropSide,)
//        lineTo(transitLanePixelStackSide)

        // move to transit lane from pixel stacks
//        lineTo(transitLaneBackDropSide)


        // park
        //lineTo(Vector2d(40.0, parkPosition.y)) // ensure that we do not clip the BB
        lineTo(parkPosition)

    }
    return sb.build()
}

fun farAuto(drive: DriveShim, alliance: Alliance, zone: PropZone): TrajectorySequence {
    // purple pixel is in the left claw, yellow is in the right

    // key points (key points are for the blue alliance)
    val spinOffset = when(alliance) {
        Alliance.RED -> 0.0001
        Alliance.BLUE -> -0.0001
    }
    val startHeading = getAllianceHeading(alliance)
    val startPose = Pose2d(-32.0, 62.0, startHeading).adjustForAlliance(alliance)
    val awayFromWallPosition = Vector2d(-36.0, 60.0).adjustForAlliance(alliance)

    val purplePixelPoseBackdropSide = Pose2d(Vector2d(-35.0, 30.0), 0.0).adjustForAlliance(alliance)
    val purplePixelPoseCenter = Pose2d(Vector2d(-36.0, 13.0), startPose.heading).adjustForAlliance(alliance)
    val purplePixelPoseAwayFromBackdrop = Pose2d(Vector2d(-44.0, 17.0), startPose.heading).adjustForAlliance(alliance)
    val purplePixelPose = when (zone) {
        PropZone.LEFT -> if(alliance== Alliance.BLUE) purplePixelPoseBackdropSide else purplePixelPoseAwayFromBackdrop
        PropZone.CENTER, PropZone.UNKNOWN -> purplePixelPoseCenter
        PropZone.RIGHT -> if(alliance== Alliance.BLUE) purplePixelPoseAwayFromBackdrop else purplePixelPoseBackdropSide
    }

    val transitLaneY = 12.0
    val nearBackDropLaneX = 36.0
    val backDropScoreX = 42.0

    val transitLanePoseAfterPurplePixel = Pose2d(Vector2d(-36.0, transitLaneY), PI + spinOffset).adjustForAlliance(alliance)
    val transitLaneBackDropSide = Vector2d(nearBackDropLaneX, transitLaneY).adjustForAlliance(alliance)
    val transitLanePixelStackSide = Vector2d(-48.0, transitLaneY).adjustForAlliance(alliance)

    val backDropScoringClawOffset = 0.0 // offset to help pixels land better if needed
    val backDropZoneSpacing = 6.0
    val backDropCenterY = 36.0
    val nearBackDropCenter = Vector2d(nearBackDropLaneX, backDropCenterY + backDropScoringClawOffset).adjustForAlliance(alliance)
    val nearBackDropLeft = Vector2d(nearBackDropLaneX, nearBackDropCenter.y + backDropZoneSpacing)
    val nearBackDropRight = Vector2d(nearBackDropLaneX, nearBackDropCenter.y - backDropZoneSpacing)
    val nearBackDropPosition = when (zone) {
        PropZone.LEFT -> nearBackDropLeft
        PropZone.CENTER, PropZone.UNKNOWN -> nearBackDropCenter
        PropZone.RIGHT -> nearBackDropRight
    }
    val backDropScoringPosition = Vector2d(backDropScoreX, nearBackDropPosition.y)  // no need to adjust for alliance (already there)
    val parkPosition = Vector2d(48.0, transitLaneY).adjustForAlliance(alliance)

    // add trajectories
    val sb = drive.trajectorySequenceBuilder(startPose)
    with(sb) {
        // move away from wall
        lineTo(awayFromWallPosition)
        // move to purple pixel
        lineToLinearHeading(purplePixelPose)
        // move to far transit lane
        lineToLinearHeading(transitLanePoseAfterPurplePixel)
        // move to back drop lane
        lineTo(transitLaneBackDropSide)
        // move to near back drop
        lineToLinearHeading(Pose2d(nearBackDropPosition, PI))


        // move to score back drop
        lineTo(backDropScoringPosition)
        lineTo(nearBackDropPosition)

        // move to transit lane to pixel stacks
//        lineTo(transitLaneBackDropSide,)
//        lineTo(transitLanePixelStackSide)

        // move to transit lane from pixel stacks
//        lineTo(transitLaneBackDropSide)

        // park
        lineTo(parkPosition)
    }

    return sb.build()
}

