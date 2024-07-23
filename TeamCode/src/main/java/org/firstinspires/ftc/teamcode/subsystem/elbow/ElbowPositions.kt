package org.firstinspires.ftc.teamcode.subsystem.elbow

import org.firstinspires.ftc.teamcode.common.config.ElbowConfig

sealed class ElbowPositions(val angleProvider: () -> Double) {
    val angle
        get() = angleProvider()

    data object ExtendedIntake: ElbowPositions({ ElbowConfig.ELBOW_EXTENDED_INTAKE })
    data object CloseIntake: ElbowPositions({ ElbowConfig.ELBOW_CLOSE_INTAKE })
    data object Travel: ElbowPositions({ ElbowConfig.ELBOW_TRAVEL })
    data object Climb: ElbowPositions({ ElbowConfig.ELBOW_CLIMB })
    data object StackIntake: ElbowPositions({ ElbowConfig.ELBOW_STACK_INTAKE })
    data object StackIntakeClose: ElbowPositions({ ElbowConfig.ELBOW_STACK_INTAKE_CLOSE })

    class Adjust(angle: Double): ElbowPositions({ angle })
}