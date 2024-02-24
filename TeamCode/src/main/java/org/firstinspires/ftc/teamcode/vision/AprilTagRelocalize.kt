package org.firstinspires.ftc.teamcode.vision

import android.util.Size
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.vision.subsystems.AprilTagPoseEstimate
import org.firstinspires.ftc.teamcode.vision.subsystems.VisionSubsystem
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilTagRelocalize(private val vision : VisionSubsystem, private val robot : Robot) : SplitSecondComponent() {
    override fun start() {
        robot.telemetry.addLine("Started april tag relocalization")
    }
    override fun loop() {
        val localizer = robot.driveBase.dt.localizer
        val aprilTagPoseEstimate = vision.aprilTagPoseEstimate
        localizer.poseEstimate = when (aprilTagPoseEstimate) {
            AprilTagPoseEstimate.None -> localizer.poseEstimate
            is AprilTagPoseEstimate.PoseEstimated -> (localizer.poseEstimate + aprilTagPoseEstimate.pose) / 2.0
        }
    }

    // Maybe make timeToRelocalize an argument to the command
    override fun isComplete() : Boolean {
        robot.telemetry.addLine("Ended april tag relocalization")
        val timeToRelocalize = 0.25
        return timer.elapsedTime > timeToRelocalize
    }
}