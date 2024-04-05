package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.util.epsilonEquals
import com.qualcomm.robotcore.util.ElapsedTime
import java.security.InvalidParameterException

/**
 * Implements a Proportional-Integral-Derivative (PID) Controller. This class provides methods to compute
 * the PID output based on target and current values, update PID coefficients, and adjust the low-pass filter's
 * cutoff frequency. It is designed to regulate a system to a desired setpoint by calculating and applying a
 * control variable.
 *
 * @param kp The proportional gain coefficient.
 * @param ki The integral gain coefficient.
 * @param kd The derivative gain coefficient.
 * @param lowPassCutoffFrequencyHz The cutoff frequency in Hz for the low-pass filter applied to the derivative term.
 * @param antiWindup The maximum allowable value for the integral term to prevent integral windup; defaults to 0.0.
 */
class PIDController(
    kp: Double,
    ki: Double,
    kd: Double,
    private var lowPassCutoffFrequencyHz: Double,
    val antiWindup: Double = 0.0,
    val errorTolerance : Double = 0.0) {

    private val timer = ElapsedTime()
    private var integral = 0.0
    private var previousError = 0.0
    private var previousDerivative = 0.0
    private var output = 0.0

    private var lastTime = timer.seconds()

    var kp = kp
        private set
    var ki = ki
        private set
    var kd = kd
        private set

    init {
        if(lowPassCutoffFrequencyHz epsilonEquals  0.0) throw InvalidParameterException("lowPassCutoffFrequencyHz cannot be 0.0.")
    }

    // Compute the PID output
    fun compute(target: Double, current: Double): Double {
        val currentTime = timer.seconds()
        val sampleTime = (currentTime - lastTime)
        lastTime = currentTime

        val error = target - current

        // Integrate only if the error is above the tolerance level
        if (Math.abs(error) > errorTolerance) {
            integral += ki * error * sampleTime

            // Anti-windup logic
            if (integral > antiWindup) integral = antiWindup
            else if (integral < -antiWindup) integral = -antiWindup
        }

        // Derivative term with low-pass filtering
        val tau = 1 / (2 * Math.PI * lowPassCutoffFrequencyHz) // Time constant of the filter
        val alpha = tau / (tau + sampleTime) // Filter coefficient
        val derivative = alpha * previousDerivative + (1 - alpha) * (error - previousError) / sampleTime

        // PID output
        output = kp * error + integral + kd * derivative

        // Update states
        previousError = error
        previousDerivative = derivative

        return output
    }

    // Update PID coefficients
    fun updateCoefficients(kp: Double, ki: Double, kd: Double) {
        this.kp = kp
        this.ki = ki
        this.kd = kd
    }

    // Update the cutoff frequency in Hz
    fun updateLowPassCutoffFrequency(frequencyHz: Double) {
        this.lowPassCutoffFrequencyHz = frequencyHz
    }
}
