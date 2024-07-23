package org.firstinspires.ftc.teamcode.subsystem.vision.detector

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

class PixelStackDetector: VisionProcessor {
    val out = Mat()
    var hsl = Mat()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
    }

    override fun processFrame(input: Mat?, captureTimeNanos: Long): Any? {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2GRAY)
        return out
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {}
}