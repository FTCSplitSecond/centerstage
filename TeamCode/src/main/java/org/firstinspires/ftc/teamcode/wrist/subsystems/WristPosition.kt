package org.firstinspires.ftc.teamcode.wrist.subsystems


sealed class WristPosition(private val angleProvider: () -> Double) {
    val angle = angleProvider()
    object ExtendedIntake : WristPosition({WristConfig.WRIST_EXTENDED_INTAKE})
    object CloseIntake : WristPosition({WristConfig.WRIST_CLOSE_INTAKE})
    object Travel : WristPosition({WristConfig.WRIST_TRAVEL})
    class Adjust(angle : Double) : WristPosition({angle})
}