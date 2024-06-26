package org.firstinspires.ftc.teamcode.telescope.subsystems


sealed class TelescopePosition(val extension : Double) {

    object ExtendedIntake : TelescopePosition(TelescopeConfig.TELESCOPE_EXTENDED_INTAKE)
    object CloseIntake : TelescopePosition(TelescopeConfig.TELESCOPE_CLOSE_INTAKE)
    object Travel : TelescopePosition(TelescopeConfig.TELESCOPE_TRAVEL)
    object Climb : TelescopePosition(TelescopeConfig.TELESCOPE_CLIMB)
    object StackIntake : TelescopePosition(TelescopeConfig.TELESCOPE_CLOSE_INTAKE)
    object StackIntakeClose : TelescopePosition(TelescopeConfig.TELESCOPE_EXTENDED_INTAKE)
    class Adjust(angle : Double) : TelescopePosition(angle)
}