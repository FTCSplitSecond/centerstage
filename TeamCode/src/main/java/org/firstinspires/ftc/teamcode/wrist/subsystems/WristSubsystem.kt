package org.firstinspires.ftc.teamcode.wrist.subsystems

import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

enum class WristPosition {
    EXTENDED_INTAKE,
    CLOSE_INTAKE,
    ADJUST,
    TRAVEL
}
class WristSubsystem(private val leftServo : Servo, private val rightServo : Servo, private val telemetry: Telemetry): Subsystem() {
    constructor(hw: HardwareManager, telemetry: Telemetry) :
            this(hw.servo("leftWristServo"),
                hw.servo("rightWristServo"),
                telemetry)

    var isTelemetryEnabled = false
    private val degreesPerMicrosecond = 180.0/2000.0
    private var movementStartTime = System.currentTimeMillis()
    var angle = getAngleFromPosition(WristPosition.TRAVEL)
        private set;

    /**
     * Angle when the current state is [WristPosition.ADJUST]
     */
    var depositAngle = 0.0

    override fun end(reason: FinishReason) {

    }

    override fun init() {
        leftServo.axonPwmRange()
        rightServo.axonPwmRange()
        rightServo.mapRange(min = 1.0, max = 0.0)
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

        leftServo goto getServoPositionFromPulseWidth(leftServoPulseWidth, leftServo)
        rightServo goto getServoPositionFromPulseWidth(rightServoPulseWidth, rightServo)
    }
    fun getServoPositionFromPulseWidth(pulseWidth : Double, servo : Servo) : Double {
        return (pulseWidth - servo.pwmRange().usPulseLower) / (servo.pwmRange().usPulseUpper - servo.pwmRange().usPulseLower)
    }
    private fun getServoPulseWidthFromAngle(angle : Double, zeroPosition: Double) : Double {
        return zeroPosition + (angle / degreesPerMicrosecond)
    }
    private fun getAngleFromPosition(position : WristPosition) : Double {
        return when(position) {
            WristPosition.TRAVEL -> WristConfig.WRIST_TRAVEL
            WristPosition.CLOSE_INTAKE -> WristConfig.WRIST_CLOSE_INTAKE
            WristPosition.EXTENDED_INTAKE -> WristConfig.WRIST_EXTENDED_INTAKE
            WristPosition.ADJUST -> depositAngle
        }
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > 100
    }

    override fun loop() {
        if(isTelemetryEnabled) {
            telemetry.addLine("Wrist: Telemetry Enabled")
            telemetry.addData("Position:", position)
            //telemetry.addData("Left Servo Position:", leftServo.position)
            //telemetry.addData("Right Servo Position:", rightServo.position)
            telemetry.update()
        }

        if (position == WristPosition.ADJUST) {
            angle = getAngleFromPosition(position)
            updateServoFromAngle(angle)
        }
    }


}