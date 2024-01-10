package org.firstinspires.ftc.teamcode.claw.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

enum class ClawPositions{
    OPEN,
    CLOSED
}
class RightClawSubsystem(private val rightServo : Servo, private val telemetry: Telemetry) : Subsystem() {
    constructor(hw: HardwareManager, telemetry: Telemetry) : this(hw.servo("rightClawServo"), telemetry)

    private var movementStartTime = System.currentTimeMillis()
    private var lastPos = 0.0
    private var isTelemetryEnabled = false

    override fun init() {
        rightServo.axonPwmRange()
    }

    var position: ClawPositions = ClawPositions.CLOSED
        set(value) {
            lastPos = getServoPositionFromPulseWidth(when(value){
                ClawPositions.OPEN -> ClawConfig.RIGHT_SERVO_OPEN_MICROSECONDS
                ClawPositions.CLOSED -> ClawConfig.RIGHT_SERVO_CLOSED_MICROSECONDS
            }, rightServo)

            rightServo goto lastPos

            movementStartTime = System.currentTimeMillis()
            field = value
        }
    private fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : Servo) : Double {
        return (pulseWidth - servo.pwmRange().usPulseLower) / (servo.pwmRange().usPulseUpper - servo.pwmRange().usPulseLower)
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > ClawConfig.estimatedTimeToComplete
    }
    override fun loop(){
        if(isTelemetryEnabled) {
            telemetry.addLine("Right Claw: Telemetry Enabled")
            telemetry.addData("right claw position", position.toString())
            telemetry.addData("rightServo.position", lastPos)
            telemetry.update()
        }
    }

    override fun end(reason: FinishReason) {

    }
}