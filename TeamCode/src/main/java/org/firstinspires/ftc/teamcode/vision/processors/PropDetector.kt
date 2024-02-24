import android.graphics.Canvas
import dev.turtles.electriceel.util.epsilonEquals
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.vision.processors.PropZoneDetected
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline


class PropDetector(val telemetry : Telemetry) : VisionProcessor {
    var zone = PropZoneDetected.NONE
        private set
    val out = Mat()
    private val HSV = Mat()
    private val white = Scalar(255.0, 255.0, 255.0)

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any {
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV)

        val leftRect = Rect(0, 100, 320, 500)
        val centerRect = Rect(320, 100, 320, 500)
        val rightRect = Rect(640, 100, 320, 500)

        Imgproc.rectangle(HSV, leftRect, white, 1)
        Imgproc.rectangle(HSV, centerRect, white, 1)
        Imgproc.rectangle(HSV, rightRect, white, 1)

        Core.extractChannel(HSV, out, 1)

        val leftCrop = out.submat(leftRect)
        val centerCrop = out.submat(centerRect)
        val rightCrop = out.submat(rightRect)

        val averages = arrayOf(Core.mean(leftCrop).`val`[0], Core.mean(centerCrop).`val`[0], Core.mean(rightCrop).`val`[0])
        val max = averages.max()

        if (max epsilonEquals averages[0]) {
            telemetry.addLine("Left")
            zone = PropZoneDetected.LEFT
        } else if (max epsilonEquals averages[1]) {
            telemetry.addLine("Center")
            zone = PropZoneDetected.CENTER
        } else {
            telemetry.addLine("Right")
            zone = PropZoneDetected.RIGHT
        }
        telemetry.update()
        return HSV
    }
    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
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
