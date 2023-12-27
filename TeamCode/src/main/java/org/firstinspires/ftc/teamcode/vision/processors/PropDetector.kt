import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core.inRange
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

enum class PropColor {
    RED,
    BLUE
}
class PropDetector(val propColor: PropColor) : VisionProcessor {
    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        // Not useful in this case, but we do need to implement it either way
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        Imgproc.cvtColor(frame!!, frame, Imgproc.COLOR_BGR2HSV)

        var lowerHsvBound = when(propColor){
            PropColor.BLUE -> Scalar(0.0, 0.0, 0.0)
            PropColor.RED -> Scalar(0.0, 0.0, 0.0)
        }
        var upperHsvBound = when(propColor){
            PropColor.BLUE -> Scalar(0.0, 0.0, 0.0)
            PropColor.RED -> Scalar(0.0, 0.0, 0.0)
        }
        inRange(frame,lowerHsvBound, upperHsvBound, frame)
        val cols = frame.cols()
        val rows = frame.rows()
        val counts = IntArray(cols) { 0 }

        for (col in 0 until cols) {
            for (row in 0 until rows) {
                val pixelValue = frame.get(row, col)
                // Counting white pixels (value of 255)
                if (pixelValue[0] > 0.0) {
                    counts[col]++
                }
            }
        }

        return null
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        // not useful here
    }

}