package org.firstinspires.ftc.teamcode.subsystem.vision

import com.acmerobotics.roadrunner.geometry.Pose2d

sealed class AprilTagPoseEstimate {
    data object None: AprilTagPoseEstimate()
    data class PoseEstimated(val pose: Pose2d, val numberOfAprilTagsDetected: Int): AprilTagPoseEstimate()
}