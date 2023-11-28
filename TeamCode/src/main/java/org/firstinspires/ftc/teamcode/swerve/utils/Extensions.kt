package org.firstinspires.ftc.teamcode.swerve.utils

import android.util.Range
import kotlin.math.*

fun Double.clamp(lower : Double, upper : Double)  = max(lower, min(upper, this))
fun Double.scale(lower : Double, upper : Double)  = (this - lower) / (upper - lower)
fun Double.compare(other : Double, epsilon : Double = 1e-6) = abs(this - other) < epsilon