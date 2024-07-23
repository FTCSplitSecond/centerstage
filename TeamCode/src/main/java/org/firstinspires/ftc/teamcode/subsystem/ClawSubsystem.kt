package org.firstinspires.ftc.teamcode.subsystem

import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.teamcode.OffseasonBot
import org.firstinspires.ftc.teamcode.common.types.ClawSide

class ClawSubsystem(
    val robot: OffseasonBot
): Subsystem() {

    enum class ClawState {
        CLOSED,
        INTERMEDIATE,
        OPEN,
        AUTO
    }

    enum class PivotState {
        FLAT,
        STORED,
        SCORING
    }

    var leftClaw = ClawState.CLOSED
    var rightClaw = ClawState.CLOSED

    lateinit var leftServo: Servo
    lateinit var rightServo: Servo

    override fun init() {
        leftServo = robot.hw.servo("leftClawServo")
        leftServo = robot.hw.servo("rightClawServo")

        leftServo.axonPwmRange()
        rightServo.axonPwmRange()
    }

    override fun loop() {}

    override fun end(reason: FinishReason) {}

    fun servo(name: String) = robot.hw.servo(name)

    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : Servo) : Double {
        return (pulseWidth - servo.pwmRange().usPulseLower) / (servo.pwmRange().usPulseUpper - servo.pwmRange().usPulseLower)
    }
}