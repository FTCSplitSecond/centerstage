package org.firstinspires.ftc.teamcode.subsystem.wrist

import org.firstinspires.ftc.teamcode.common.config.WristConfig

sealed class WristPositions(private val angleProvider: () -> Double) {
    val angle = angleProvider()

    data object ExtendedIntake: WristPositions({ WristConfig.WRIST_EXTENDED_INTAKE })
    data object CloseIntake: WristPositions({ WristConfig.WRIST_CLOSE_INTAKE })
    data object Travel: WristPositions({ WristConfig.WRIST_TRAVEL })

    class Adjust(angle: Double): WristPositions({ angle })
}