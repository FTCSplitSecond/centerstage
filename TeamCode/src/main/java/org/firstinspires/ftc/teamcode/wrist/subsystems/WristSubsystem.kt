package org.firstinspires.ftc.teamcode.wrist.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.InverseKinematics
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class WristPosition {
    EXTENDED_INTAKE,
    CLOSE_INTAKE,
    DEPOSIT,
    TRAVEL
}
class WristSubsystem(private val leftServo : ServoImplEx, private val rightServo : ServoImplEx, private val telemetry: Telemetry): SubsystemBase() {
    constructor(robot: Robot) :
            this(robot.hardwareMap.get(ServoImplEx::class.java, "leftWristServo"),
                robot.hardwareMap.get(ServoImplEx::class.java, "rightWristServo"),
                robot.telemetry)
    var isTelemetryEnabled = false
    private val degreesPerMicrosecond = 180.0/2000.0
    private var movementStartTime = System.currentTimeMillis()
    var angle = getAngleFromPosition(WristPosition.TRAVEL)
        private set;
    init {
        register()
        leftServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        rightServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        rightServo.direction = Servo.Direction.REVERSE
    }
    var position: WristPosition = WristPosition.TRAVEL
    set(value) {
        if(value != position) {
            angle = getAngleFromPosition(value)
            updateServoFromAngle(angle)
            field = value
            movementStartTime = System.currentTimeMillis()
        }
    }
    fun updateServoFromAngle(angle: Double) {
        val leftServoPulseWidth = getServoPulseWidthFromAngle(angle, WristConfig.LEFT_SERVO_ZERO_POSITION)
        val rightServoPulseWidth = getServoPulseWidthFromAngle(angle, WristConfig.RIGHT_SERVO_ZERO_POSITION)

        leftServo.position = getServoPositionFromPulseWidth(leftServoPulseWidth, leftServo)
        rightServo.position = getServoPositionFromPulseWidth(rightServoPulseWidth, rightServo)
    }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : ServoImplEx) : Double {
        return (pulseWidth - servo.pwmRange.usPulseLower) / (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower)
    }
    private fun getServoPulseWidthFromAngle(angle : Double, zeroPosition: Double) : Double {
        return zeroPosition + (angle / degreesPerMicrosecond)
    }
    private fun getAngleFromPosition(position : WristPosition) : Double {
        return when(position) {
            WristPosition.TRAVEL -> WristConfig.WRIST_TRAVEL
            WristPosition.CLOSE_INTAKE -> WristConfig.WRIST_CLOSE_INTAKE
            WristPosition.EXTENDED_INTAKE -> WristConfig.WRIST_EXTENDED_INTAKE
            WristPosition.DEPOSIT -> InverseKinematics.calculateArmInverseKinematics(pixelLevel).wristAngle
        }
    }
    var pixelLevel : Int = 0
        set (value){
            if(position == WristPosition.DEPOSIT) {
                angle = InverseKinematics.calculateArmInverseKinematics(value).wristAngle
                updateServoFromAngle(angle)
            }
            field = value
        }


    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > 100
    }

    override fun periodic() {
        if(isTelemetryEnabled) {
            telemetry.addLine("Wrist: Telemetry Enabled")
            telemetry.addData("Position:", position)
            telemetry.addData("Left Servo Position:", leftServo.position)
            telemetry.addData("Right Servo Position:", rightServo.position)
            telemetry.update()
        }
    }


}