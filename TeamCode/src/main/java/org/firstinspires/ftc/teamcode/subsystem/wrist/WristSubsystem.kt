package org.firstinspires.ftc.teamcode.subsystem.wrist

import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.teamcode.SplitSecondBot
import org.firstinspires.ftc.teamcode.common.config.WristConfig

class WristSubsystem(val robot: SplitSecondBot, r: HardwareManager): Subsystem() {
    val wristServo = r.servo("wristServo")

    var angle = WristPositions.Travel.angle
        private set

    private val degreesPerMicrosecond = -180.0/2000.0
    private var movementStartTime = System.currentTimeMillis()

    var position: WristPositions = WristPositions.Travel
        set(value) {
            if (value != position) {
                angle = value.angle
                updateServoFromAngle(angle)
                field = value
                movementStartTime = System.currentTimeMillis()
            }
        }

    init {
        wristServo.axonPwmRange()
    }

    fun updateServoFromAngle(angle: Double) {
        val wristServoPulse = getServoPulseFromAngle(angle, WristConfig.WRIST_SERVO_ZERO_POSITION)
        wristServo goto getServoPositionFromPulse(wristServoPulse, wristServo)
    }

    private fun getServoPositionFromPulse(pulse: Double, servo: Servo): Double {
        return (pulse - servo.pwmRange().usPulseLower) / (servo.pwmRange().usPulseUpper - servo.pwmRange().usPulseLower)
    }

    private fun getServoPulseFromAngle(angle: Double, zeroPosition: Double): Double {
        return zeroPosition + (angle / degreesPerMicrosecond)
    }

    override fun init() {}

    override fun loop() {}

    override fun end(reason: FinishReason) {}

    val shouldBeComplete: Boolean
        get() = System.currentTimeMillis() - movementStartTime > 100
}