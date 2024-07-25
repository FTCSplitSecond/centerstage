package org.firstinspires.ftc.teamcode.telescope.commands

import android.util.Log
import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.telescope.subsystems.*

open class SetTelescopePosition(val telescope: TelescopeSubsystem, private val position: TelescopePosition) : Component() {
    override fun end(reason: FinishReason) {}

    override fun isComplete() : Boolean {
        val timedOut = timer.elapsedTime > 0.25
        if(timedOut) Log.d("telescope","timeout")
        return telescope.isAtTarget() || timedOut
    }

    override fun loop() {}

    override fun start() {
        telescope.position = position
    }
}

