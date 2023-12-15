package org.firstinspires.ftc.teamcode.wrist.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class WristPositions{
    REST,
    UP,
    DOWN
}
class WristSubsystem(private val leftServo : ServoImplEx, private val rightServo : ServoImplEx, private val telemetry: Telemetry): SubsystemBase() {
    constructor(robot: Robot) :
            this(robot.hardwareMap.get(ServoImplEx::class.java, "leftWristServo"),
                robot.hardwareMap.get(ServoImplEx::class.java, "rightWristServo"),
                robot.telemetry)

    companion object{
        const val LEFT_SERVO_RESTING_POSITION = 1384.0
        const val RIGHT_SERVO_RESTING_POSITION = 1502.0
        //TODO: Find actual values for these
        const val LEFT_SERVO_UP_POSITION = 500.0
        const val RIGHT_SERVO_UP_POSITION = 500.0
        const val LEFT_SERVO_DOWN_POSITION = 2500.0
        const val RIGHT_SERVO_DOWN_POSITION = 2500.0

    }
    private var movementStartTime = System.currentTimeMillis()

    init {
        register()
    }
    var position: WristPositions = WristPositions.REST
    set(value) {
            leftServo.position = getServoPositionFromPulseWidth(when(value){
                WristPositions.REST -> LEFT_SERVO_RESTING_POSITION
                WristPositions.UP -> LEFT_SERVO_UP_POSITION
                WristPositions.DOWN -> LEFT_SERVO_DOWN_POSITION
            }, leftServo)
            rightServo.position = getServoPositionFromPulseWidth(when(value){
                WristPositions.REST -> RIGHT_SERVO_RESTING_POSITION
                WristPositions.UP -> RIGHT_SERVO_UP_POSITION
                WristPositions.DOWN -> RIGHT_SERVO_DOWN_POSITION
            }, rightServo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : ServoImplEx) : Double {
        return (pulseWidth - servo.pwmRange.usPulseLower) / (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower)
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > 100
    }

    private var isTelemetryEnabled = false

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