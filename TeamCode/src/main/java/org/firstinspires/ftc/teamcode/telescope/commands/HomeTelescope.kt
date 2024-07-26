package org.firstinspires.ftc.teamcode.telescope.commands

import android.util.Log
import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsystem

class HomeTelescope(val telescope: TelescopeSubsystem) : Component() {
    override fun end(reason: FinishReason) {}

    override fun isComplete() : Boolean {
        val timedOut = timer.elapsedTime > 3.0 // give it 3 seconds to home
        if(timedOut) Log.d("telescope","homing timeout")
        return !telescope.isHoming || timedOut
    }

    override fun loop() {}

    override fun start() {
        telescope.isHoming = true
    }
}