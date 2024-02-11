package org.firstinspires.ftc.teamcode.elbow.subsystems


sealed class ElbowPosition(val angle : Double) {
    object ExtendedIntake : ElbowPosition(ElbowConfig.ELBOW_EXTENDED_INTAKE)
    object CloseIntake : ElbowPosition(ElbowConfig.ELBOW_CLOSE_INTAKE)
    object Travel : ElbowPosition(ElbowConfig.ELBOW_TRAVEL)
    object Climb : ElbowPosition(ElbowConfig.ELBOW_CLIMB)
    object StackIntake : ElbowPosition(ElbowConfig.ELBOW_STACK_INTAKE)
    object StackIntakeClose : ElbowPosition(ElbowConfig.ELBOW_STACK_INTAKE_CLOSE)
    class Adjust(angle : Double) : ElbowPosition(angle)
}