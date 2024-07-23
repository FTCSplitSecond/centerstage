package org.firstinspires.ftc.teamcode.subsystem.vision.detector

import android.graphics.Canvas
import dev.turtles.electriceel.util.epsilonEquals
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class PropDetector(): VisionProcessor {
    var zone = PropZoneDetected.NONE
        private set

    val out = Mat()
    val hsv = Mat()
    val white = Scalar(255.0, 255.0, 255.0)

    override fun init(p0: Int, p1: Int, p2: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV)

        val leftRect = Rect(0, 100, 320, 500)
        val centerRect = Rect(320, 100, 320, 500)
        val rightRect = Rect(640, 100, 320, 500)

        Imgproc.rectangle(hsv, leftRect, white, 1)
        Imgproc.rectangle(hsv, centerRect, white, 1)
        Imgproc.rectangle(hsv, rightRect, white, 1)

        Core.extractChannel(hsv, out, 1)

        val leftCrop = out.submat(leftRect)
        val centerCrop = out.submat(centerRect)
        val rightCrop = out.submat(rightRect)

        val averages = arrayOf(Core.mean(leftCrop).`val`[0], Core.mean(centerCrop).`val`[0], Core.mean(rightCrop).`val`[0])
        val max = averages.max()

        if (max epsilonEquals averages[0]) {
            zone = PropZoneDetected.LEFT
        } else if (max epsilonEquals averages[1]) {
            zone = PropZoneDetected.CENTER
        } else {
            zone = PropZoneDetected.RIGHT
        }

        return hsv
    }

    override fun onDrawFrame(p0: Canvas?, p1: Int, p2: Int, p3: Float, p4: Float, p5: Any?) {}
}