package org.firstinspires.ftc.teamcode.subsystem

import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.teamcode.OffseasonBot
import org.firstinspires.ftc.teamcode.common.config.ClawConfig
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

    fun updateState(side: ClawSide, state: ClawState) {
        val position = getClawStatePosition(side, state)

        when (side) {
            ClawSide.LEFT -> {
                leftServo goto position
                leftClaw = state
            }
            ClawSide.RIGHT -> {
                rightServo goto position
                rightClaw = state
            }
            ClawSide.BOTH -> {
                leftServo goto getClawStatePosition(ClawSide.LEFT, state)
                leftClaw = state
                rightServo goto getClawStatePosition(ClawSide.RIGHT, state)
                rightClaw = state
            }
        }
    }

    fun getClawStatePosition(side: ClawSide, state: ClawState): Double {
        return when (side) {
            ClawSide.LEFT -> {
                when (state) {
                    ClawState.CLOSED -> ClawConfig.LEFT_CLAW_CLOSED
                    ClawState.INTERMEDIATE -> ClawConfig.LEFT_CLAW_INTERMEDIATE
                    ClawState.OPEN -> ClawConfig.LEFT_CLAW_OPEN
                    ClawState.AUTO -> ClawConfig.LEFT_CLAW_AUTO
                }
            }
            ClawSide.RIGHT -> {
                when (state) {
                    ClawState.CLOSED -> ClawConfig.RIGHT_CLAW_CLOSED
                    ClawState.INTERMEDIATE -> ClawConfig.RIGHT_CLAW_INTERMEDIATE
                    ClawState.OPEN -> ClawConfig.RIGHT_CLAW_OPEN
                    ClawState.AUTO -> ClawConfig.RIGHT_CLAW_AUTO
                }
            }
            else -> 0.5
        }
    }

    fun getExactState(side: ClawSide): ClawState {
        if (side == ClawSide.BOTH) {
            return if (robot.claw.rightClaw === ClawState.CLOSED || robot.claw.leftClaw === ClawState.CLOSED) ClawState.CLOSED else ClawState.OPEN
        }
        return if (side == ClawSide.LEFT) leftClaw else rightClaw
    }

    override fun loop() {}

    override fun end(reason: FinishReason) {}

    fun servo(name: String) = robot.hw.servo(name)

    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : Servo) : Double {
        return (pulseWidth - servo.pwmRange().usPulseLower) / (servo.pwmRange().usPulseUpper - servo.pwmRange().usPulseLower)
    }
}