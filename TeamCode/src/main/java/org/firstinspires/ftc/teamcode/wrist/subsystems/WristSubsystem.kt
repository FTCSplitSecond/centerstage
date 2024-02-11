package org.firstinspires.ftc.teamcode.wrist.subsystems

import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class WristSubsystem(private val wristServo: Servo, private val telemetry: Telemetry): Subsystem() {
    constructor(hw: HardwareManager, telemetry: Telemetry) :
            this(hw.servo("wristServo"),
                telemetry)

    var isTelemetryEnabled = false
    private val degreesPerMicrosecond = -180.0/2000.0
    private var movementStartTime = System.currentTimeMillis()
    var angle = WristPosition.Travel.angle

        private set;


    override fun end(reason: FinishReason) {

    }

    override fun init() {
        wristServo.axonPwmRange()
    }
    var position: WristPosition = WristPosition.Travel
    set(value) {
        if(value != position) {
            angle = position.angle
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


    fun movementShouldBeComplete() : Boolean {
        return System.currentTimeMillis() - movementStartTime > 100
    }

    override fun loop() {
        if(isTelemetryEnabled) {
            telemetry.addLine("Wrist: Telemetry Enabled")
            telemetry.addData("Position:", position)
            //telemetry.addData("Wrist Servo Position:", wristServo.position)
            telemetry.update()
        }
    }
}