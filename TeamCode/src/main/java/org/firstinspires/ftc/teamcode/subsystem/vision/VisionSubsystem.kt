package org.firstinspires.ftc.teamcode.subsystem.vision

import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.SplitSecondBot
import org.firstinspires.ftc.teamcode.subsystem.vision.detector.PropDetector
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import android.util.Size
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.firstinspires.ftc.teamcode.subsystem.vision.detector.PropZoneDetected
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

class VisionSubsystem(val robot: SplitSecondBot): Subsystem() {
    private val aprilTagProcessor = AprilTagProcessor.Builder()
        .setLensIntrinsics(1530.08, 1522.20, 1082.787, 551.41) // camera for "webcam1" c922
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .build()

    private val propDetector = PropDetector()

    private val backCamPortal = VisionPortal.Builder()
        .setCamera(robot.hwMap.get(WebcamName::class.java, "webcam1"))
        .setCameraResolution(Size(1920, 1080))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .addProcessors(aprilTagProcessor, propDetector)
        .enableLiveView(true)
        .setAutoStopLiveView(true)
        .build()

    var aprilTagEstimate: AprilTagPoseEstimate = AprilTagPoseEstimate.None
        private set

    var propZoneDetected = PropZoneDetected.NONE
        private set

    private var isPropEnabled = true

    fun stopProp() {
        isPropEnabled = false
        backCamPortal.setProcessorEnabled(propDetector, false)
    }

    override fun init() {}

    override fun loop() {
        val detections = aprilTagProcessor.detections
        aprilTagEstimate = if (detections.size > 0) {
            val poseEstimates = detections.map { d -> getPoseEstimateFromDetection(d) }
            AprilTagPoseEstimate.PoseEstimated(
                Pose2d(
                    poseEstimates.sumOf { it.x } / detections.size,
                    poseEstimates.sumOf { it.y } / detections.size,
                    poseEstimates.sumOf { it.heading } / detections.size
                ), detections.size
            )
        } else AprilTagPoseEstimate.None

        propZoneDetected = if (isPropEnabled) propDetector.zone else PropZoneDetected.NONE
    }

    private fun getPoseEstimateFromDetection(detection: AprilTagDetection): Pose2d {
        val tagLocation = Vector2D(
            detection.metadata.fieldPosition.get(0).toDouble(),
            detection.metadata.fieldPosition.get(1).toDouble()
        )

        val cameraToTag = Vector2D(
            calibrateDistance(-detection.ftcPose.y),
            detection.ftcPose.x
        )

        val robotCenterToCamera = Vector2D(-6.5, 4.0)
        val robotEstimatedPosition: Vector2D = tagLocation.add(cameraToTag).add(robotCenterToCamera)
        val headingEstimate: Double = 180.0 - detection.ftcPose.yaw

        return Pose2d(robotEstimatedPosition.x, robotEstimatedPosition.y, headingEstimate)
    }

    fun calibrateDistance(x: Double): Double {
        return  x / 1.1 - 0.25
    }

    override fun end(reason: FinishReason) {
        backCamPortal.close()
    }
}