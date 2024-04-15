package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.util.epsilonEquals
import com.qualcomm.robotcore.util.ElapsedTime
import java.security.InvalidParameterException

class LowPassFilter(val lowPassCutoffFrequencyHz: Double, initialValue : Double) {
    private val timer = ElapsedTime()
    private var lastTime = timer.seconds()
    var filteredValue = initialValue
        private set

    init {
        if(lowPassCutoffFrequencyHz epsilonEquals  0.0) throw InvalidParameterException("lowPassCutoffFrequencyHz cannot be 0.0.")
    }
    val tau = 1.0 / (2.0 * Math.PI * lowPassCutoffFrequencyHz) // Time constant of the filter

    fun compute(newValue : Double) : Double {
        val currentTime = timer.seconds()
        val sampleTime = (currentTime - lastTime)
        lastTime = currentTime

        val alpha = tau / (tau + sampleTime) // Filter coefficient
        filteredValue = alpha * filteredValue + (1.0 - alpha) * newValue
        return  filteredValue
    }
}