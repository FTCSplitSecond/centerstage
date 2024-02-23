package org.firstinspires.ftc.teamcode.vision.processors

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class PixelStackDetector(val telemetry: Telemetry) : OpenCvPipeline() {
    val out = Mat()
    var HSL = Mat()
    override fun processFrame(input: Mat?): Mat {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2GRAY)
        return out
    }
}