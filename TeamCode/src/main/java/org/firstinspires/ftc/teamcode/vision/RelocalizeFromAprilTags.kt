package org.firstinspires.ftc.teamcode.vision

import android.util.Log
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.vision.subsystems.AprilTagPoseEstimate

class RelocalizeFromAprilTags(val robot: Robot) : SplitSecondComponent() {
    override fun start() {
        val aprilTagPoseEstimate = robot.vision.aprilTagPoseEstimate
        when(aprilTagPoseEstimate) {
            is AprilTagPoseEstimate.PoseEstimated -> {
                robot.driveBase.dt.poseEstimate = aprilTagPoseEstimate.pose
                Log.d("relocalize", String.format("x=%.2f, y=%.2f, heading=%.2f, detections=%d",
                    aprilTagPoseEstimate.pose.x, aprilTagPoseEstimate.pose.y, aprilTagPoseEstimate.pose.heading,
                    aprilTagPoseEstimate.numberOfAprilTagsDetected))
            }
            is AprilTagPoseEstimate.None -> { }
        }
    }

}