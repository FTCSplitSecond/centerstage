package org.firstinspires.ftc.teamcode.utilities

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx

class AxonServo(hw: HardwareMap,
                servoName: String,
                minimumPulseWidth: Double,
                maximumPulseWidth: Double,
                private val maximumRotationRangeDegrees :Double = 180.0,
                private val isReversed: Boolean = false) {

    private val servo: ServoImplEx

    init {
        servo = hw.get(ServoImplEx::class.java, servoName)
        servo.pwmRange = PwmControl.PwmRange(minimumPulseWidth, maximumPulseWidth)
    }

    val minimumAngle = -maximumRotationRangeDegrees / 2
    val maximumAngle = maximumRotationRangeDegrees / 2
    val servoMiddlePosition = 0.5

    var angle : Double
        get() = getAngleDegreesFromServoPosition(servo.position)
        set(angle) {
            if(isReversed)
                servo.position = getServoPositionFromAngleDegrees(-angle)
            else
                servo.position = getServoPositionFromAngleDegrees(angle)
        }

    val position : Double
        get() = servo.position
    fun getServoPositionFromAngleDegrees(angle : Double) : Double {
        return clamp(angle/maximumRotationRangeDegrees + servoMiddlePosition, 0.0, 1.0)
    }

    fun getAngleDegreesFromServoPosition(servoPosition: Double) : Double {
        return (clamp(servoPosition, 0.0, 1.0) - servoMiddlePosition) * maximumRotationRangeDegrees
    }
    fun getServoPulseWidthFromAngleDegrees(angle : Double) : Double {
        val servoPosition = getServoPositionFromAngleDegrees(angle)
        return getServoPulseWidthFromPosition(servoPosition)
    }
    fun getServoPulseWidthFromPosition(position : Double) : Double {
        return position * (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower) + servo.pwmRange.usPulseLower
    }
}