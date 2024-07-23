package org.firstinspires.ftc.teamcode.subsystem.wrist

class WristPositions(private val angleProvider: () -> Double) {
    val angle = angleProvider()

}