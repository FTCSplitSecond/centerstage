import dev.turtles.electriceel.util.epsilonEquals
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

enum class PropZone {
    LEFT,
    CENTER,
    RIGHT,
    UNKNOWN
}
class PropDetector(val telemetry : Telemetry) : OpenCvPipeline() {

    var zone = PropZone.UNKNOWN
    val out = Mat()
    val HSV = Mat()
    val black = Scalar(0.0, 0.0, 0.0)

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV)

        val leftRect = Rect(0, 0, 320, 720)
        val centerRect = Rect(320, 0, 320, 720)
        val rightRect = Rect(640, 0, 320, 720)

//            input.copyTo(out)

        Imgproc.rectangle(HSV, leftRect, black, 1)
        Imgproc.rectangle(HSV, centerRect, black, 1)
        Imgproc.rectangle(HSV, rightRect, black, 1)

        Core.extractChannel(HSV, out, 1)

        val leftCrop = out.submat(leftRect)
        val centerCrop = out.submat(centerRect)
        val rightCrop = out.submat(rightRect)

//            Core.extractChannel(leftCrop, leftCrop, 1)
//            Core.extractChannel(rightCrop, rightCrop, 1)

        val averages = arrayOf(Core.mean(leftCrop).`val`[0], Core.mean(centerCrop).`val`[0], Core.mean(rightCrop).`val`[0])
        val max = averages.max()

        if (max epsilonEquals averages[0]) {
            telemetry.addLine("Left")
            zone = PropZone.LEFT
        } else if (max epsilonEquals averages[1]) {
            telemetry.addLine("Center")
            zone = PropZone.CENTER
        } else {
            telemetry.addLine("Right")
            zone = PropZone.RIGHT
        }
        telemetry.update()

        return HSV
    }
}
