package org.firstinspires.ftc.teamcode.DroneLaunch.Commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.DroneLaunch.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.DroneLaunch.Subsystems.DroneSubsystem

class SetDroneForInit(private val setDrone : DroneSubsystem): CommandBase(){
    init {
        addRequirements(setDrone)
    }

    override fun initialize() {
        setDrone.position = DronePositions.LAUNCH

    }

    override fun isFinished(): Boolean {
        return setDrone.movementShouldBeComplete()
    }
}