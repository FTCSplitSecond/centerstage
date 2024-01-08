package org.firstinspires.ftc.teamcode.telescope.commands

import com.arcrobotics.ftclib.command.CommandBase
import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.telescope.subsystems.*

open class SetTelescopePosition(val telescope: TelescopeSubsytem, private val position: TelescopePosition) : Component() {
    override fun end(reason: FinishReason) {}

    override fun isComplete() = telescope.isAtTarget()

    override fun loop() {}

    override fun start() {
        telescope.position = position
    }
}

