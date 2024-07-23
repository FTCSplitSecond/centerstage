package org.firstinspires.ftc.teamcode.component.drone

import org.firstinspires.ftc.teamcode.common.component.SplitSecondComponent
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem

class SetDroneForInit(private val drone: DroneSubsystem): SplitSecondComponent() {
    override fun start() {
        drone.launched = false
        drone.pitchPosition = DroneSubsystem.PitchPositions.STOWED
    }
}