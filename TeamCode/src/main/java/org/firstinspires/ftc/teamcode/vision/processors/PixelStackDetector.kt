package org.firstinspires.ftc.teamcode.vision.processors

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class PixelStackDetector(val telemetry: Telemetry) : VisionProcessor {
    val out = Mat()
    var HSL = Mat()
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
    ) {
    }
}