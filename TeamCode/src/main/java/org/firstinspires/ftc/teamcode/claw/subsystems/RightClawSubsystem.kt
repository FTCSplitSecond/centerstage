package org.firstinspires.ftc.teamcode.claw.subsystems

import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.util.OpModeType

enum class ClawPositions{
    OPEN,
    CLOSED,
    DROP
}
class RightClawSubsystem(private val robot : Robot, private val rightServo : Servo, private val telemetry: Telemetry) : Subsystem() {
    constructor(robot : Robot, hw: HardwareManager, telemetry: Telemetry) : this(robot, hw.servo("rightClawServo"), telemetry)

    private var movementStartTime = System.currentTimeMillis()
    private var lastPos = 0.0
    private var isTelemetryEnabled = false

    override fun init() {
    }

    var position: ClawPositions = ClawPositions.CLOSED
        set(value) {
            updateServoFromPosition(value)
            movementStartTime = System.currentTimeMillis()
            field = value
        }

    init {
        rightServo.axonPwmRange()
        updateServoFromPosition(position)
    }

    private fun updateServoFromPosition(position : ClawPositions) {
        lastPos = getServoPositionFromPulseWidth(when(position){
            ClawPositions.OPEN -> ClawConfig.RIGHT_SERVO_OPEN_MICROSECONDS
            ClawPositions.CLOSED -> when (robot.opModeType) {
                OpModeType.TELEOP -> ClawConfig.RIGHT_SERVO_CLOSED_TELEOP_MICROSECONDS
                OpModeType.AUTONOMOUS -> ClawConfig.RIGHT_SERVO_CLOSED_AUTO_MICROSECONDS
            }
            ClawPositions.DROP -> ClawConfig.RIGHT_SERVO_DROP_MICROSECONDS
        }, rightServo)

        rightServo goto lastPos
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