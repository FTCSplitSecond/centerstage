package org.firstinspires.ftc.teamcode.vision

import android.util.Size
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilTagRelocalizer(val robot : Robot) : SplitSecondComponent() {
    companion object {
        lateinit var tagProcessor : AprilTagProcessor
        lateinit var visionPortal: VisionPortal
        fun initAprilTag(webcamName : WebcamName) {
            val tagProcessor = AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(1530.08, 1522.20, 1082.787, 551.41) // camera for "webcam 1"
                .build()

            visionPortal = VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(Size(1920, 1080))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .addProcessor(tagProcessor)
                .build()
            visionPortal.setProcessorEnabled(tagProcessor, true)
        }
    }
    override fun start() {
        val telemetry = robot.telemetry
        val currentDetections: List<AprilTagDetection> = tagProcessor.detections
        telemetry.addLine(String.format("# AprilTags Detected %d", currentDetections.size))

        // Step through the list of detections and display info for each one.

        // Step through the list of detections and display info for each one.
        for (detection in currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(
                    String.format(
                        "\n==== (ID %d) %s",
                        detection.id,
                        detection.metadata.name
                    )
                )
                telemetry.addLine(
                    String.format(
                        "XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z
                    )
                )
                telemetry.addLine(
                    String.format(
                        "PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw
                    )
                )
                telemetry.addLine(
                    String.format(
                        "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                    )
                )
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id))
                telemetry.addLine(
                    String.format(
                        "Center %6.0f %6.0f   (pixels)",
                        detection.center.x,
                        detection.center.y
                    )
                )
            }
        } // end for() loop


        // Add "key" information to telemetry

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.")
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)")
        telemetry.addLine("RBE = Range, Bearing & Elevation")

//        TODO: robot.driveBase.dt.poseEstimate = april tag location - april tag xyz estimate
    }
}