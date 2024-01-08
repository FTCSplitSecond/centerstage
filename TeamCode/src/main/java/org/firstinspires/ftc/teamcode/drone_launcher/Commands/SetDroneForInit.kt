package org.firstinspires.ftc.teamcode.drone_launcher.Commands

import com.arcrobotics.ftclib.command.CommandBase
import dev.turtles.anchor.component.Component
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem

class SetDroneForInit(private val setDrone : DroneSubsystem): SplitSecondComponent(){
    override fun start() {
        setDrone.position = DronePositions.LAUNCH
    }

    override fun isComplete() = setDrone.movementShouldBeComplete()
}