import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.vision.processors.PropDetectorConfig
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.*
import org.opencv.imgproc.*

enum class PropColor {
    RED,
    BLUE
}
enum class PropZone {
    LEFT,
    CENTER,
    RIGHT,
    UNKNOWN
}
class PropDetector(val propColor: PropColor) : VisionProcessor {
    var propZone = PropZone.UNKNOWN
        private set
    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        // Not useful in this case, but we do need to implement it either way
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        Imgproc.cvtColor(frame!!, frame, Imgproc.COLOR_BGR2HSV)

        // for red, we may need to wrap around the 180 mark and add the two ranges together (second range is H 0-10)
        // for blue, we just need to check the one range
        val lowerHsvBound = when(propColor){
            PropColor.BLUE -> Scalar(PropDetectorConfig.BLUE_S_MIN, PropDetectorConfig.BLUE_S_MIN, PropDetectorConfig.BLUE_V_MIN)
            PropColor.RED -> Scalar(PropDetectorConfig.RED_S_MIN, PropDetectorConfig.RED_S_MIN, PropDetectorConfig.RED_V_MIN)
//            PropColor.RED -> Scalar(0.0, 100.0, 100.0)
        }
        val upperHsvBound = when(propColor){
            PropColor.BLUE -> Scalar(PropDetectorConfig.BLUE_H_MAX, PropDetectorConfig.BLUE_S_MAX, PropDetectorConfig.BLUE_V_MAX)
            PropColor.RED -> Scalar(PropDetectorConfig.RED_H_MAX, PropDetectorConfig.RED_S_MAX, PropDetectorConfig.RED_V_MAX)
//            PropColor.RED -> Scalar(10.0, 100.0, 100.0)
        }
        Core.inRange(frame,lowerHsvBound, upperHsvBound, frame)

        val leftZoneRect = when(propColor){
            PropColor.BLUE -> Rect(PropDetectorConfig.BLUE_LEFT_X, PropDetectorConfig.BLUE_LEFT_Y, PropDetectorConfig.RECT_WIDTH, PropDetectorConfig.RECT_HEIGHT)
            PropColor.RED -> Rect(PropDetectorConfig.RED_LEFT_X, PropDetectorConfig.RED_LEFT_Y, PropDetectorConfig.RECT_WIDTH, PropDetectorConfig.RECT_HEIGHT)
        }
        val centerZoneRect = when(propColor){
            PropColor.BLUE -> Rect(PropDetectorConfig.BLUE_CENTER_X, PropDetectorConfig.BLUE_CENTER_Y, PropDetectorConfig.RECT_WIDTH, PropDetectorConfig.RECT_HEIGHT)
            PropColor.RED -> Rect(PropDetectorConfig.RED_CENTER_X, PropDetectorConfig.RED_CENTER_Y, PropDetectorConfig.RECT_WIDTH, PropDetectorConfig.RECT_HEIGHT)
        }

        val leftZoneMat = frame.submat(leftZoneRect)
        val centerZoneMat = frame.submat(centerZoneRect)

        val leftZoneCount = Core.countNonZero(leftZoneMat)
        val centerZoneCount = Core.countNonZero(centerZoneMat)

        // draw a blue rectangle around the left zone and the center zone
        val boxColorRGB = Scalar(0.0, 0.0, 255.0)
        Imgproc.rectangle(frame, leftZoneRect, boxColorRGB)
        Imgproc.rectangle(frame, centerZoneRect, boxColorRGB)

        propZone = if(leftZoneCount > PropDetectorConfig.COUNT_THRESHOLD){
            PropZone.LEFT
        } else if(centerZoneCount > PropDetectorConfig.COUNT_THRESHOLD){
            PropZone.CENTER
        } else {
            PropZone.RIGHT
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