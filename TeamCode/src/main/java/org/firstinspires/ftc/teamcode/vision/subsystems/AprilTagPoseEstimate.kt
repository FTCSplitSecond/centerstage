package org.firstinspires.ftc.teamcode.vision.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d

sealed class AprilTagPoseEstimate {
    object None : AprilTagPoseEstimate()
    data class PoseEstimated(val pose : Pose2d, val numberOfAprilTagsDetected : Int) : AprilTagPoseEstimate()

}