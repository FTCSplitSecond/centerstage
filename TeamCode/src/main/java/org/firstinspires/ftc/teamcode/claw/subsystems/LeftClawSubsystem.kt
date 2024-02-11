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
import org.firstinspires.ftc.teamcode.robot.util.OpModeType


class LeftClawSubsystem(private val robot : Robot, private val hw: HardwareManager, private val leftServo : Servo, private val telemetry: Telemetry) : Subsystem() {
    constructor(robot: Robot, hw: HardwareManager, telemetry: Telemetry) : this(robot, hw, hw.servo("leftClawServo"), telemetry)

    private var movementStartTime = System.currentTimeMillis()
    private var lastPos = 0.0
    var isTelemetryEnabled = false

    override fun init() {
    }

    var position: ClawPositions = ClawPositions.CLOSED
        set(value) {
            updateServoFromPosition(value)
            movementStartTime = System.currentTimeMillis()
            field = value
        }

    init {
        leftServo.axonPwmRange() // = PwmControl.PwmRange(500.0, 2500.0)
        updateServoFromPosition(position)
    }

    private fun updateServoFromPosition(position : ClawPositions) {
        val lastPos = getServoPositionFromPulseWidth(when (position) {
            ClawPositions.OPEN -> ClawConfig.LEFT_SERVO_OPEN_MICROSECONDS
            ClawPositions.CLOSED -> when (robot.opModeType) {
                OpModeType.TELEOP -> ClawConfig.LEFT_SERVO_CLOSED_TELEOP_MICROSECONDS
                OpModeType.AUTONOMOUS -> ClawConfig.LEFT_SERVO_CLOSED_AUTO_MICROSECONDS
            }
            ClawPositions.DROP -> ClawConfig.LEFT_SERVO_DROP_MICROSECONDS
        }, leftServo)

        leftServo goto lastPos
    }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : Servo) : Double {
        return (pulseWidth - servo.pwmRange().usPulseLower) / (servo.pwmRange().usPulseUpper - servo.pwmRange().usPulseLower)
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > ClawConfig.estimatedTimeToComplete
    }

    override fun loop(){
        if(isTelemetryEnabled) {
            telemetry.addLine("Left Claw: Telemetry Enabled")
            telemetry.addData("left claw position", position.toString())
            telemetry.addData("leftServo.position", lastPos)
        }
    }

    override fun end(reason: FinishReason) {

    }
}