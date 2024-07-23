package org.firstinspires.ftc.teamcode.component.drone

import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import org.firstinspires.ftc.teamcode.common.component.SplitSecondComponent
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem

class LaunchDrone(private val drone: DroneSubsystem): SplitSecondComponent() {
    override fun start() {
        series(
            instant { drone.pitchPosition = DroneSubsystem.PitchPositions.LAUNCHED },
            delay(0.25),
            instant { drone.launched = true },
            delay(0.25),
            instant { drone.pitchPosition = DroneSubsystem.PitchPositions.STOWED }
        )
    }
}