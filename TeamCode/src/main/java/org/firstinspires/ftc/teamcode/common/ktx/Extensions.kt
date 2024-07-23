package org.firstinspires.ftc.teamcode.common.ktx

import dev.turtles.electriceel.util.epsilonEquals
import kotlin.math.sign

fun Double.adjustForKStatic(kStatic: Double): Double {
    val basePower = this

    return if (basePower epsilonEquals 0.0)
        0.0
    else
        sign(basePower) * kStatic + basePower
}