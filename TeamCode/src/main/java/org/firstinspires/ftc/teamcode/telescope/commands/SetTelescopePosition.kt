package org.firstinspires.ftc.teamcode.telescope.commands

import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeState
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsystem

open class SetTelescopePosition(val telescope: TelescopeSubsystem, private val position: Double) : Component() {
    constructor(telescope: TelescopeSubsystem, state: TelescopeState) : this(telescope,
        stateToPosition(state))

    companion object {
        fun stateToPosition(state : TelescopeState) : Double {
            return when (state) {
                TelescopeState.TRAVEL -> TelescopeConfig.TELESCOPE_TRAVEL
                TelescopeState.CLOSE_INTAKE -> TelescopeConfig.TELESCOPE_CLOSE_INTAKE
                TelescopeState.ADJUST -> TelescopeConfig.TELESCOPE_ADJUST
                TelescopeState.EXTENDED_INTAKE -> TelescopeConfig.TELESCOPE_EXTENDED_INTAKE
            }
        }
    }

    override fun end(reason: FinishReason) {}

    override fun isComplete() = telescope.isAtTarget()

    override fun loop() {}

    override fun start() {
        telescope.targetExtensionInches = position
    }
}

