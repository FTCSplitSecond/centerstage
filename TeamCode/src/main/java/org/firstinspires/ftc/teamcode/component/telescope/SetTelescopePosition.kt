package org.firstinspires.ftc.teamcode.component.telescope

import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.subsystem.telescope.TelescopePositions
import org.firstinspires.ftc.teamcode.subsystem.telescope.TelescopeSubsystem

class SetTelescopePosition(val telescope: TelescopeSubsystem, val position: TelescopePositions): Component() {
    override fun start() {
        telescope.position = position
    }

    override fun loop() {}

    override fun end(reason: FinishReason) {}

    override fun isComplete(): Boolean {
        val timedOut = timer.elapsedTime > 0.25
        if (timedOut) println("timed out")
        return telescope.inThresh || timedOut
    }
}