package org.firstinspires.ftc.teamcode.elbow.commands

import android.util.Log
import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem

class HomeElbow(val elbow : ElbowSubsystem) : Component() {
    override fun end(reason: FinishReason) {}

    override fun isComplete() : Boolean {
        val timedOut = timer.elapsedTime > 3.0 // give it 3 seconds to home
        if(timedOut) Log.d("elbow","homing timeout")
        return !elbow.isHoming || timedOut
    }

    override fun loop() {}

    override fun start() {
        elbow.isHoming = true
    }
}