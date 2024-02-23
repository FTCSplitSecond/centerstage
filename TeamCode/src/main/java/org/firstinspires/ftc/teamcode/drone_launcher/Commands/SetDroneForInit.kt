package org.firstinspires.ftc.teamcode.drone_launcher.Commands

import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.TriggerPositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.PitchPositions

class SetDroneForInit(private val setDrone : DroneSubsystem): SplitSecondComponent(){
    override fun start() {
        setDrone.triggerPosition = TriggerPositions.HELD
        setDrone.pitchPosition = PitchPositions.STOWED
    }

    override fun isComplete() = setDrone.movementShouldBeComplete()
}