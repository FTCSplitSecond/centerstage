package org.firstinspires.ftc.teamcode.drone_launcher.Subsystems

import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.claw.subsystems.DroneConfig

enum class DronePositions{
    HELD,
    LAUNCH
}

class DroneSubsystem(private val servo : Servo, private val telemetry: Telemetry) : Subsystem() {
    constructor(hw: HardwareManager, telemetry: Telemetry) : this(hw.servo("droneServo"), telemetry)

    private var movementStartTime = System.currentTimeMillis()

    override fun init() {
        servo.axonPwmRange()
//        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    var position: DronePositions = DronePositions.HELD
        set(value) {
            servo goto getServoPositionFromPulseWidth(when (value) {
                DronePositions.HELD -> DroneConfig.HELD_MICROSECONDS
                DronePositions.LAUNCH -> DroneConfig.LAUNCH_MICROSECONDS
            }, servo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }

    fun getServoPositionFromPulseWidth(pulseWidth: Double, servo: Servo): Double {
        return (pulseWidth - servo.pwmRange().usPulseLower) / (servo.pwmRange().usPulseUpper - servo.pwmRange().usPulseLower)
    }

    fun movementShouldBeComplete(): Boolean {
        return System.currentTimeMillis() - movementStartTime > DroneConfig.estimatedTimeToComplete
    }

    private var isTelemetryEnabled = false
    override fun loop() {
        if (isTelemetryEnabled) {

            telemetry.update()
        }
    }


    override fun end(reason: FinishReason) {
    }

}