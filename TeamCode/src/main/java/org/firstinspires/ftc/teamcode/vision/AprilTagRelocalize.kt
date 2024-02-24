package org.firstinspires.ftc.teamcode.vision

import android.util.Size
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.vision.subsystems.VisionSubsystem
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilTagRelocalize(private val vision : VisionSubsystem, private val robot : Robot) : SplitSecondComponent() {
    private lateinit var aprilTag : AprilTagProcessor
    private lateinit var visionPortal: VisionPortal
    override fun start() {
        aprilTag =
            AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(1530.08, 1522.20, 1082.787, 551.41)
                .build()
        val builder = VisionPortal.Builder()

        builder.setCamera(vision.getWebcamName())

        builder.setCameraResolution(Size(1920, 1080))

        builder.enableLiveView(true)

        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2)

        builder.setAutoStopLiveView(false)

        builder.addProcessor(aprilTag)

        visionPortal = builder.build()

        visionPortal.setProcessorEnabled(aprilTag, true)
    }

    override fun loop() {
        val detections = aprilTag.detections ?: return
        val localizer = robot.driveBase.dt.localizer

        val tagPoseEstimates = ArrayList<Pose2d>()
        for (detection in detections) {
            if (detection.metadata != null) {
                val tagLocation = Vector2D(
                    detection.metadata.fieldPosition[0].toDouble(),
                    detection.metadata.fieldPosition[1].toDouble()
                )
                val cameraToTag = Vector2D(-detection.ftcPose.y / 1.1, detection.ftcPose.x)
                val robotCenterToCamera = Vector2D(-6.5, 4.0)
                val robotEstimatedPosition = tagLocation.add(cameraToTag.add(robotCenterToCamera))
                val headingEstimate = 180 - detection.ftcPose.yaw
                val robotPoseEstimate = Pose2d(
                    robotEstimatedPosition.x,
                    robotEstimatedPosition.y,
                    headingEstimate
                )
                tagPoseEstimates.add(robotPoseEstimate)
            }
        }

        var sumTagPoseEstimates = Pose2d()
        tagPoseEstimates.forEach {sumTagPoseEstimates += it}
        val avgTagPoseEstimate = sumTagPoseEstimates / tagPoseEstimates.size.toDouble()
        localizer.poseEstimate = (localizer.poseEstimate + avgTagPoseEstimate) / 2.0
    }

    // Maybe make timeToRelocalize an argument to the command
    override fun isComplete() : Boolean {
        val timeToRelocalize = 0.25
        return timer.elapsedTime > timeToRelocalize
    }
}