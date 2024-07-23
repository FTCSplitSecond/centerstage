package org.firstinspires.ftc.teamcode.subsystem.telescope

import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig

sealed class TelescopePositions(val extension: Double) {
    data object ExtendedIntake: TelescopePositions(TelescopeConfig.TELESCOPE_EXTENDED_INTAKE)
    data object CloseIntake: TelescopePositions(TelescopeConfig.TELESCOPE_CLOSE_INTAKE)
    data object Travel: TelescopePositions(TelescopeConfig.TELESCOPE_TRAVEL)
    data object Climb: TelescopePositions(TelescopeConfig.TELESCOPE_CLIMB)
    data object StackIntake: TelescopePositions(TelescopeConfig.TELESCOPE_CLOSE_INTAKE)
    data object StackIntakeClose: TelescopePositions(TelescopeConfig.TELESCOPE_CLOSE_INTAKE)

    class Adjust(extension: Double): TelescopePositions(extension)
}