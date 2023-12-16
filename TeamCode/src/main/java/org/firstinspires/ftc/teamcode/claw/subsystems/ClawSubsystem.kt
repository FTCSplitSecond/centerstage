package org.firstinspires.ftc.teamcode.claw.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.util.Timing.Timer
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class ClawPositions{
    OPEN,
    CLOSED
}
class ClawSubsystem(private val leftServo : ServoImplEx, private val rightServo : ServoImplEx, private val telemetry: Telemetry) : SubsystemBase() {
    constructor(robot: Robot) :
            this(robot.hardwareMap.get(ServoImplEx::class.java, "leftClawServo"),
                 robot.hardwareMap.get(ServoImplEx::class.java, "rightClawServo"),
                 robot.telemetry)
    companion object {
        const val LEFT_SERVO_OPEN_MICROSECONDS = 500.0
        const val LEFT_SERVO_CLOSED_MICROSECONDS = 1450.0
        const val RIGHT_SERVO_OPEN_MICROSECONDS = 2500.0
        const val RIGHT_SERVO_CLOSED_MICROSECONDS = 1525.0
        const val estimatedTimeToComplete = 100 // 100 ms based on https://axon-robotics.com/products/micro
    }
    private var movementStartTime = System.currentTimeMillis()
    init {
        register()
        leftServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        rightServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }
    var position:ClawPositions = ClawPositions.CLOSED
        set(value) {
            leftServo.position = getServoPositionFromPulseWidth(when(value){
                ClawPositions.OPEN -> LEFT_SERVO_OPEN_MICROSECONDS
                ClawPositions.CLOSED -> LEFT_SERVO_CLOSED_MICROSECONDS
            }, leftServo)
            rightServo.position = getServoPositionFromPulseWidth(when(value){
                ClawPositions.OPEN -> RIGHT_SERVO_OPEN_MICROSECONDS
                ClawPositions.CLOSED -> RIGHT_SERVO_CLOSED_MICROSECONDS
            }, rightServo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : ServoImplEx) : Double {
        return (pulseWidth - servo.pwmRange.usPulseLower) / (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower)
    }
    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > estimatedTimeToComplete
    }
    private var isTelemetryEnabled = false
    override fun periodic(){
        if(isTelemetryEnabled) {
            telemetry.addLine("Claw: Telemetry Enabled")
            telemetry.addData("claw position", position.toString())
            telemetry.addData("leftServo.position", leftServo.position)
            telemetry.addData("rightServo.position", rightServo.position)
            telemetry.update()
        }
    }

}