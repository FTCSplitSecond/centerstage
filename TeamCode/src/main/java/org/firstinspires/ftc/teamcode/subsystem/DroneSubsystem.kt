package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager

class DroneSubsystem(r: HardwareManager): Subsystem() {
    var launched = false

    private val droneServo = r.servo("droneServo")
    private val pitchServo = r.servo("dronePitch")

    enum class PitchPositions(val position: Double) {
        STOWED(0.0),
        LAUNCHED(0.0)
    }

    var pitchPosition = PitchPositions.STOWED
        set(value) {
            field = value
            pitchServo goto value.position
        }

    override fun init() {
        launched = false
    }

    override fun loop() {
        droneServo goto if (launched) closed else open
    }

    override fun end(reason: FinishReason) {}

    @Config
    companion object DroneConstants {
        @JvmField var closed: Double = 0.0
        @JvmField var open: Double = 0.0
    }
}