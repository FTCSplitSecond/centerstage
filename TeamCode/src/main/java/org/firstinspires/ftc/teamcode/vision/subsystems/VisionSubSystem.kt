package org.firstinspires.ftc.teamcode.vision.subsystems

import PropDetector
import android.util.Size
import com.acmerobotics.roadrunner.geometry.Pose2d
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.vision.processors.PropZoneDetected
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class VisionSubsystem(private val robot: Robot): Subsystem() {

    private val aprilTagDetector = AprilTagProcessor.Builder()
        .setLensIntrinsics(1530.08, 1522.20, 1082.787, 551.41) // camera for "webcam1" c922
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .build()
    private val propDetector = PropDetector(robot.telemetry)
    private val backCamPortal = VisionPortal.Builder()
        .setCamera(robot.hardwareMap.get(WebcamName::class.java, "webcam1"))
        .setCameraResolution(Size(1920, 1080))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .addProcessors(aprilTagDetector, propDetector)
        .enableLiveView(true)
        .setAutoStopLiveView(true)
        .build()
    init {
        // only run these in auto
        if(robot.opModeType == OpModeType.AUTONOMOUS) {
            backCamPortal.setProcessorEnabled(aprilTagDetector, true)
            backCamPortal.setProcessorEnabled(propDetector, true)
        } else {
            backCamPortal.setProcessorEnabled(aprilTagDetector, false)
            backCamPortal.setProcessorEnabled(propDetector, false)
        }
    }
    var aprilTagPoseEstimate : AprilTagPoseEstimate = AprilTagPoseEstimate.None
        private set
    var propZoneDetected  = PropZoneDetected.NONE
        private set
    private var isPropZoneDetectorEnabled = true
    fun disablePropZoneDetector() {
        isPropZoneDetectorEnabled = false
        backCamPortal.setProcessorEnabled(propDetector, false)
    }
    override fun init() {
    }
    override fun end(reason: FinishReason) {
    }
    override fun loop() {
        // update the aprilTags pose (if aprilTags are available
        val detections = aprilTagDetector.detections
        aprilTagPoseEstimate =  if(detections.size > 0) {
            val poseEstimates = detections.map { d -> getPoseEstimateFromDetection(d) }
            val averageX = poseEstimates.sumOf{it.x}/detections.size
            AprilTagPoseEstimate.PoseEstimated(
                Pose2d(
                    poseEstimates.sumOf{it.x}/detections.size,
                    poseEstimates.sumOf{it.y}/detections.size,
                poseEstimates.sumOf{it.heading}/detections.size),detections.size)
        } else AprilTagPoseEstimate.None

        // update the propZone detected
        propZoneDetected = if(isPropZoneDetectorEnabled) propDetector.zone else PropZoneDetected.NONE
    }
    fun getPoseEstimateFromDetection(detection : AprilTagDetection) : Pose2d {
        val tagLocation = Vector2D(
            detection.metadata.fieldPosition.get(0).toDouble(),
            detection.metadata.fieldPosition.get(1).toDouble()
        )
        val cameraToTag = Vector2D(calibrateDistance(-detection.ftcPose.y), detection.ftcPose.x)
        val robotCenterToCamera = Vector2D(-6.5, 4.0)
        val robotEstimatedPosition: Vector2D = tagLocation.add(cameraToTag.add(robotCenterToCamera))
        val headingEstimate: Double = 180 - detection.ftcPose.yaw
        return Pose2d(robotEstimatedPosition.x, robotEstimatedPosition.y, headingEstimate)
    }
    fun calibrateDistance(x : Double) : Double {
        // calibration data gathered empirically
        return x/1.1 - 0.25
    }

}