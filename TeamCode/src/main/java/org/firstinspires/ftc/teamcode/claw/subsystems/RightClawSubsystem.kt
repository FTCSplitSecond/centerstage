package org.firstinspires.ftc.teamcode.claw.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class ClawPositions{
    OPEN,
    CLOSED
}
class RightClawSubsystem( private val rightServo : ServoImplEx, private val telemetry: Telemetry) : SubsystemBase() {
    constructor(robot: Robot) :
            this(robot.hardwareMap.get(ServoImplEx::class.java, "rightClawServo"),
                 robot.telemetry)
    private var movementStartTime = System.currentTimeMillis()
    init {
        register()
        rightServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }
    var position: ClawPositions = ClawPositions.CLOSED
        set(value) {
            rightServo.position = getServoPositionFromPulseWidth(when(value){
                ClawPositions.OPEN -> ClawConfig.RIGHT_SERVO_OPEN_MICROSECONDS
                ClawPositions.CLOSED -> ClawConfig.RIGHT_SERVO_CLOSED_MICROSECONDS
            }, rightServo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : ServoImplEx) : Double {
        return (pulseWidth - servo.pwmRange.usPulseLower) / (servo.pwmRange.usPulseUpper - servo.pwmRange.usPulseLower)
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > ClawConfig.estimatedTimeToComplete
    }
    private var isTelemetryEnabled = false
    override fun periodic(){
        if(isTelemetryEnabled) {
            telemetry.addLine("Right Claw: Telemetry Enabled")
            telemetry.addData("right claw position", position.toString())
            telemetry.addData("rightServo.position", rightServo.position)
            telemetry.update()
        }
    }

}