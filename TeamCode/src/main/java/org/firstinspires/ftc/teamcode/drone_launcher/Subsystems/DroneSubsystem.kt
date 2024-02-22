package org.firstinspires.ftc.teamcode.drone_launcher.Subsystems


import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import dev.turtles.electriceel.wrapper.interfaces.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneConfig

enum class TriggerPositions{
    HELD,
    RELEASE
}

enum class PitchPositions {
    STOWED,
    LAUNCH
}
class DroneSubsystem(private val triggerServo : Servo, private  val pitchServo: Servo, private val telemetry: Telemetry) : Subsystem() {
    constructor(hw: HardwareManager, telemetry: Telemetry) : this(hw.servo("droneServo"), hw.servo("dronePitch"), telemetry)

    private var movementStartTime = System.currentTimeMillis()

    override fun init() {
        triggerServo.axonPwmRange()
        pitchServo.axonPwmRange()
//        triggerServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    var triggerPosition: TriggerPositions = TriggerPositions.HELD
        set(value) {
            triggerServo goto getTriggerServoPositionFromPulseWidth(when (value) {
                TriggerPositions.HELD -> DroneConfig.HELD_MICROSECONDS
                TriggerPositions.RELEASE -> DroneConfig.RELEASE_MICROSECONDS
            }, triggerServo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }
    var pitchPosition: PitchPositions = PitchPositions.STOWED
        set(value) {
            pitchServo goto getPitchServoPositionFromPulseWidth(when (value) {
                PitchPositions.STOWED -> DroneConfig.STOWED
                PitchPositions.LAUNCH -> DroneConfig.LAUNCH
            }, pitchServo)
            movementStartTime = System.currentTimeMillis()
            field = value
        }

    fun getTriggerServoPositionFromPulseWidth(pulseWidth: Double, triggerServo: Servo): Double {
        return (pulseWidth - triggerServo.pwmRange().usPulseLower) / (triggerServo.pwmRange().usPulseUpper - triggerServo.pwmRange().usPulseLower)
    }
    fun getPitchServoPositionFromPulseWidth(pulseWidth: Double, pitchServo: Servo): Double {
        return (pulseWidth - pitchServo.pwmRange().usPulseLower) / (pitchServo.pwmRange().usPulseUpper - triggerServo.pwmRange().usPulseLower)
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