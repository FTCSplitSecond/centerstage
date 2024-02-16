package org.firstinspires.ftc.teamcode.drone_launcher.Commands

import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem

class SetDroneForInit(private val setDrone : DroneSubsystem): SplitSecondComponent(){
    override fun start() {
        setDrone.position = DronePositions.HELD
    }

    override fun isComplete() = setDrone.movementShouldBeComplete()
}