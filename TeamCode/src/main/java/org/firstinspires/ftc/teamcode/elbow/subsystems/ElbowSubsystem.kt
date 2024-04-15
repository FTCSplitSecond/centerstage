package org.firstinspires.ftc.teamcode.elbow.subsystems

import com.acmerobotics.roadrunner.profile.AccelerationConstraint
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.profile.VelocityConstraint
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import org.apache.commons.math3.util.FastMath.pow
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.B_VISCOUS_DAMPING_COEFFICIENT
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_HOME
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_PID_ANTI_WINDUP_LIMIT
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_PID_KD
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_PID_KI
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_PID_KP
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_PID_LP_HZ
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KA_MULTIPLIER
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KG_MULTIPLIER
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KK_VOLTS
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KS_OMEGA_THRESHOLD
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KS_POWER_THRESHOLD
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KS_VOLTS
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KV_MULTIPLIER
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.swerve.utils.clamp
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem
import org.firstinspires.ftc.teamcode.util.LowPassFilter
import org.firstinspires.ftc.teamcode.util.PIDController
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sign
import kotlin.math.sqrt


class ElbowSubsystem(private val robot: Robot, private val hw : HardwareManager, val telescope: TelescopeSubsytem) : Subsystem() {

    var isTelemetryEnabled = true
    private val motor = hw.motor("elbow")
    var isEnabled = true
        set(value) {
            field = value
            if (!value)
                motor power 0.0
        }


    override fun init() {
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        motor.brake(false)
    }


    val ELBOW_MOTOR_PPR = 751.8 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val DEGREES_PER_REVOLUTION = 360.0*(14.0/53.0)  //degrees
    val PIDTolerance = 1.0 // degrees
    var targetAngle : Double = 0.0
        private set
    val motionProfileTimer = ElapsedTime()
    var previousTarget = targetAngle
    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentAngle, 0.0, 0.0),
        MotionState(targetAngle, 0.0, 0.0),
        ElbowVelocityConstraint(currentAngle, targetAngle),
        ElbowAccelerationConstraint(currentAngle, targetAngle))
    var powerOverride = 0.0

    var deltaTimer = ElapsedTime()
    private var angularX : Double = currentAngle
    private var angularV : Double = 0.0
    private var angularA : Double = 0.0
    private var voltage = hw.voltage()
    private var lastTargetAngularX = motionProfile[0.0].x

    private fun getEncoderTicksFromAngle(angle : Double) : Double {
        return angle/ DEGREES_PER_REVOLUTION * ELBOW_MOTOR_PPR // ticks
    }
    private fun getAngleFromEncoderTicks(encoderTicks : Double) : Double {
        return  encoderTicks/ ELBOW_MOTOR_PPR * DEGREES_PER_REVOLUTION + ELBOW_HOME // degrees
    }

    val currentAngle : Double
        get() {
            return getAngleFromEncoderTicks(motor.encoder.getCounts())
        }

    var position : ElbowPosition = ElbowPosition.Travel
        set(value) {
            targetAngle = value.angle
            field = value
        }

    private val controller = PIDController(ELBOW_PID_KP, ELBOW_PID_KI, ELBOW_PID_KD, ELBOW_PID_LP_HZ, ELBOW_PID_ANTI_WINDUP_LIMIT, 0.2)
//    private val angularXLowPassFilter = LowPassFilter(ELBOW_PID_LP_HZ, angularX)
    private val angularVLowPassFilter = LowPassFilter(ELBOW_PID_LP_HZ, angularV)
    private val angularALowPassFilter = LowPassFilter(ELBOW_PID_LP_HZ, angularA)
    private val voltageLowPassFilter = LowPassFilter(1.0, voltage)

    fun isAtTarget() : Boolean {
        return Math.abs(targetAngle-currentAngle)<PIDTolerance
    }


    override fun loop() {

        // update pid values from config only if they have changed
        if (!(controller.kp epsilonEquals  ELBOW_PID_KP && controller.ki epsilonEquals  ELBOW_PID_KI && controller.kd epsilonEquals  ELBOW_PID_KD ))
            controller.updateCoefficients(ELBOW_PID_KP, ELBOW_PID_KI, ELBOW_PID_KD)

        val deltaT = deltaTimer.seconds()
        deltaTimer.reset()
        val newAngularX = currentAngle
        val newAngularV = (newAngularX - angularX) / deltaT
        val newAngularA = (newAngularV - angularV) / deltaT
        angularX = newAngularX
        angularV = angularVLowPassFilter.compute(newAngularV)
        angularA = angularALowPassFilter.compute(newAngularA)
        voltage = voltageLowPassFilter.compute(hw.voltage())

        // using the current motion profile target as the starting point for the next motion profile
        // this is to ensure continuity between motion profiles (eliminates jitter where the motor as moved past current angle
        // and the motion profile will generate a first location that is backwards from the current direction of motion)
        val currentMotionProfileX = motionProfile[motionProfileTimer.seconds()].x
        val clampedTarget = targetAngle.clamp(
            ElbowConfig.ELBOW_MIN,
            ElbowConfig.ELBOW_MAX)
        generateMotionProfile(clampedTarget, currentMotionProfileX, angularV, angularA)
        val motionProfileTarget = motionProfile[motionProfileTimer.seconds()]


        val jt = getMoment(telescope.currentExtensionInches)
        val gravityFeedForward = getGravityFeedforward(angularX, jt)

        val targetAngularX = motionProfileTarget.x
        val targetAngularV = motionProfileTarget.v
        val targetAngularA = motionProfileTarget.a
        val feedforwardVAPower = getVAFeedForwardPower(targetAngularV, targetAngularA, jt)
        val pidPower = controller.compute(lastTargetAngularX, angularX)
        val basePower = feedforwardVAPower + pidPower
        val frictionPower = getFrictionAdjustment(targetAngularV, basePower)
        val totalPower = if (powerOverride epsilonEquals 0.0) basePower + frictionPower + gravityFeedForward else powerOverride + gravityFeedForward
        val adjustedPower = adjustForBatteryVoltage(totalPower, voltage)

        if (isEnabled)
            motor power adjustedPower

        lastTargetAngularX = targetAngularX

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow  : Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle Degree:", targetAngle)
            robot.telemetry.addData("Current Angle Degree", currentAngle)
            robot.telemetry.addData("feedforwardVAPower", feedforwardVAPower)
            robot.telemetry.addData("gravityFeedForward", gravityFeedForward)
            robot.telemetry.addData("pidPower", pidPower)
            robot.telemetry.addData("voltage", voltage)
            robot.telemetry.addData("angularX", angularX)
            robot.telemetry.addData("angularV", angularV)
            robot.telemetry.addData("angularA", angularA)
            robot.telemetry.addData("targetAngularX", targetAngularX)
            robot.telemetry.addData("targetAngularV", targetAngularV)
            robot.telemetry.addData("targetAngularA", targetAngularA)
            robot.telemetry.addData("frictionPower", frictionPower)
            robot.telemetry.addData("totalPower", totalPower)
            robot.telemetry.addData("adjustedPower", adjustedPower)
            robot.telemetry.addData("Angle Error Degree", targetAngle - currentAngle)
            robot.telemetry.addData("Telescope Ext", robot.telescope.currentExtensionInches)
            robot.telemetry.addData("Is At Target", this.isAtTarget())
            robot.telemetry.addData("motor.getCurrrent (mA)", motor.getCurrent() * 1000)
        }
    }

    companion object {
        private val motorStallTorque = 1.83260 // Nm for a GoBilda 5203 Yellow Jacket Motor @ 435 rpm
        private val gearRatio = 14.0/53.0 // 14:53 gear ratio
        private val motorStallCurrent = 9.2 // A for a GoBilda 5203 Yellow Jacket Motor @ 435 rpm
        private val motorNominalVoltage = 12.0 // V for a GoBilda 5203 Yellow Jacket Motor @ 435 rpm
        private val motorResistance = motorNominalVoltage / motorStallCurrent // Ohms
        private val motorKt = motorStallTorque / motorStallCurrent // Nm/A
        private val g = 9.8 // m/s^2 (gravity)
        private val nominalVoltage = 12.0

        fun getMoment(extInches: Double ) : Double {
            val rx = extInches * 0.0254

            // quadratic coefficient values from excel fit
            val A = 0.21618950
            val B = 0.23259825
            val C = 0.05603656
            val rt = sqrt(A * pow(rx, 2.0) + B * rx + C) // rt is the Center of Mass of the arm (from Excel fit)
            val m = 0.608383140 // mass of the arm in kg (estimated in Excel)
            val jt = m * pow(rt,2.0) // moment of inertia of the arm about the elbow joint
            return jt
        }

        fun getGravityFeedforward(thetaDegrees: Double, jt: Double) : Double {
            val theta = Math.toRadians(thetaDegrees)
            val kg = KG_MULTIPLIER * cos(theta) * jt * motorResistance * gearRatio * g / motorKt
            return kg / nominalVoltage
        }

        /**
         * Calculates the nominal power needed for a given target angular velocity and acceleration
         *
         * @param omegaDegrees target angular velocity in deg/s
         * @param alphaDegrees target angular acceleration in deg/s^2
         * @param jt telescope moment of inertia
         *
         **/
        fun getVAFeedForwardPower(omegaDegrees : Double, alphaDegrees : Double, jt: Double ) : Double {
            val omega = Math.toRadians(omegaDegrees)
            val alpha = Math.toRadians(alphaDegrees)

            val b = B_VISCOUS_DAMPING_COEFFICIENT
            // all of these are in Volts (will convert to nominal power later)
            val ka = KA_MULTIPLIER * motorResistance * gearRatio * jt / motorKt
            val kv = KV_MULTIPLIER * (motorKt/ gearRatio +  motorResistance * gearRatio * b / motorKt)
            val voltage = ka * alpha + kv * omega
            return voltage / nominalVoltage
        }

        /**
         * Calculates the friction adjustment needed for a given angular velocity
         * (includes static and kinetic friction)
         *
         * @param omegaDegrees current angular velocity in deg/s
         * @param power current requested power
         */
        fun getFrictionAdjustment(omegaDegrees : Double, power : Double) : Double {
            val ks = KS_VOLTS // Volts to overcome static friction
            val kk = KK_VOLTS // Volts to overcome kinetic friction
            val omega = Math.toRadians(omegaDegrees)
            val thresholdOmega = Math.toRadians(KS_OMEGA_THRESHOLD) // rad/s
            val voltage =   if (abs(power) > KS_POWER_THRESHOLD)
                                sign(power) * (kk + (ks - kk) * exp(-pow(omega / thresholdOmega,2))) // friction model
                            else 0.0
            return voltage / nominalVoltage
        }
        fun adjustForBatteryVoltage(power : Double, currentVoltage : Double) : Double {
            return (power * nominalVoltage / currentVoltage).clamp(-1.0, 1.0)
        }
    }
    override fun end(reason: FinishReason) {

    }
    private fun generateMotionProfile(target: Double, currentX: Double, currentV: Double, currentA: Double) {
        if (!(previousTarget epsilonEquals target)) {
            previousTarget = target

            motionProfile = MotionProfileGenerator.generateMotionProfile(
                MotionState(currentX, currentV, currentA),
                MotionState(target, 0.0, 0.0),
                ElbowVelocityConstraint(currentX, target),
                ElbowAccelerationConstraint(currentX, target))

            motionProfileTimer.reset()
        }
    }
    class ElbowVelocityConstraint(private val start : Double, private val target : Double) : VelocityConstraint {
        override fun get(s: Double): Double {
            return ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY
        }
    }
    class ElbowAccelerationConstraint(private val start : Double, private val target : Double) : AccelerationConstraint {
        override fun get(s: Double): Double {
            val distance = abs(target - start)
            val percentPath = 100.0 * (s - start)/(target - start)
            val direction = sign(target - start)

            // this creates an asymmetric acceleration profile...possibly add one for when it is terminating around 0 degrees (near the floor)
            return if(distance < 20.0)
                ElbowConfig.ELBOW_MAX_ANGULAR_ACCELERATION
            else if (direction > 0.0) {
                if (s < 90.0) ElbowConfig.ELBOW_MAX_ANGULAR_ACCELERATION
                else ElbowConfig.ELBOW_MAX_ANGULAR_DECELERATION
            } else {
                if (s > 90.0) ElbowConfig.ELBOW_MAX_ANGULAR_ACCELERATION
                else ElbowConfig.ELBOW_MAX_ANGULAR_DECELERATION
            }
        }
    }
}