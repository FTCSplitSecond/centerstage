package org.firstinspires.ftc.teamcode.Telescope.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.Telescope.subsystems.*

class SetTelescopePosition(private val telescope : TelescopeSubsytem, private val position: TelescopePosition) : CommandBase() {

    init {
        addRequirements(telescope)
    }

    override fun initialize() {
        telescope.position = position
    }

    override fun isFinished(): Boolean {
        return telescope.isAtTarget()
    }

}

