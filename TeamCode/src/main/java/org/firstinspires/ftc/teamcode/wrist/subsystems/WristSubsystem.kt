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
    TRAVEL,
    PREDEPOSIT
}
class WristSubsystem(private val wristServo : Servo,  private val telemetry: Telemetry): Subsystem() {
    constructor(hw: HardwareManager, telemetry: Telemetry) :
            this(hw.servo("wristServo"),
                telemetry)

    var isTelemetryEnabled = false
    private val degreesPerMicrosecond = -180.0/2000.0
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
        wristServo.axonPwmRange()
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
        val wristServoPulseWidth = getServoPulseWidthFromAngle(angle, WristConfig.LEFT_SERVO_ZERO_POSITION)

        wristServo goto getServoPositionFromPulseWidth(wristServoPulseWidth, wristServo)
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
            WristPosition.PREDEPOSIT -> WristConfig.WRIST_PREDEPOSIT
        }
    }

    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > 100
    }

    override fun loop() {
        if(isTelemetryEnabled) {
            telemetry.addLine("Wrist: Telemetry Enabled")
            telemetry.addData("Position:", position)
            //telemetry.addData("servo Position:", wristServo.position)
            telemetry.update()
        }

        if (position == WristPosition.ADJUST) {
            angle = getAngleFromPosition(position)
            updateServoFromAngle(angle)
        }
    }


}