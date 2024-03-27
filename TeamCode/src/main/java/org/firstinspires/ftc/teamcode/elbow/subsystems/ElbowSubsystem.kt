package org.firstinspires.ftc.teamcode.elbow.subsystems

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.epsilonEquals
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.wrapper.HardwareManager
import org.apache.commons.math3.util.FastMath.pow
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_HOME
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_ACCELERATION
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.swerve.utils.clamp
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sign
import kotlin.math.sqrt


class ElbowSubsystem(private val robot: Robot, private val hw : HardwareManager, val telescope: TelescopeSubsytem) : Subsystem() {

    var isEnabled = true
    var isTelemetryEnabled = true
    private val motor = hw.motor("elbow")


    override fun init() {
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }


    val ELBOW_MOTOR_PPR = 751.8 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val DEGREES_PER_REVOLUTION = 360.0*(14.0/53.0)  //degrees
    val PIDTolerance = 5.0 // degrees
    var targetAngle : Double = 0.0
        private set
    val motionProfileTimer = ElapsedTime()
    var previousTarget = targetAngle
    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentAngle, 0.0, 0.0),
        MotionState(targetAngle, 0.0, 0.0),
        { ELBOW_MAX_ANGULAR_VELOCITY },
        { ELBOW_MAX_ANGULAR_ACCELERATION },
    )

    var deltaTimer = ElapsedTime()
    private var angularX : Double = currentAngle
    private var angularV : Double = 0.0
    private var angularA : Double = 0.0
    private var lastMotionProfileTarget = motionProfile[0.0]

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

    private val controller = PIDController(ElbowConfig.ELBOW_KP, ElbowConfig.ELBOW_KI, ElbowConfig.ELBOW_KD)


    fun isAtTarget() : Boolean {
        return Math.abs(targetAngle-currentAngle)<PIDTolerance
    }


    override fun loop() {
        val deltaT = deltaTimer.seconds()
        deltaTimer.reset()
        val newAngularX = currentAngle
        val newAngularV = (newAngularX - angularX) / deltaT
        val newAngularA = (newAngularV - angularV) / deltaT
        angularX = newAngularX
        angularV = newAngularV
        angularA = newAngularA

        // using the current motion profile target as the starting point for the next motion profile
        // this is to ensure continuity between motion profiles (eliminates jitter where the motor as moved past current angle
        // and the motion profile will generate a first location that is backwards from the current direction of motion)
        val currentMotionProfileX = motionProfile[motionProfileTimer.seconds()].x
        val clampedTarget = targetAngle.clamp(
            ElbowConfig.ELBOW_MIN,
            ElbowConfig.ELBOW_MAX)
        generateMotionProfile(clampedTarget, currentMotionProfileX, angularV, angularA)
        val motionProfileTarget = motionProfile[motionProfileTimer.seconds()]

        if (isEnabled) {
            val feedforwardPower = getFeedForwardPower(currentAngle, motionProfileTarget.v, motionProfileTarget.a, telescope.currentExtensionInches)

            // use the lastMotionProfileTarget as the setpoint as we are using the PID to correct errors beyond the feedforward
            val pidPower = controller.calculate(currentAngle, lastMotionProfileTarget.x)
            val basePower = pidPower + feedforwardPower
            val frictionPower = getFrictionAdjustment(Math.toRadians(angularV), basePower)
            val totalPower = basePower + frictionPower
            val adjustedPower = adjustForBatteryVoltage(totalPower, hw.voltage())
            motor power adjustedPower
        } else motor power 0.0

        lastMotionProfileTarget = motionProfileTarget

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow  : Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle Degree:", targetAngle)
            robot.telemetry.addData("Current Angle Degree", currentAngle)
            robot.telemetry.addData("Angle Error Degree", targetAngle - currentAngle)
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

        /**
         * Calculates the nominal power needed for a given target angular velocity and acceleration
         * (includes gravity compensation)
         *
         * @param thetaDegrees current angle in degrees
         * @param omegaDegrees target angular velocity in deg/s
         * @param alphaDegrees target angular acceleration in deg/s^2
         * @param extInches current telescope extension in inches
         *
         **/
        fun getFeedForwardPower(thetaDegrees: Double, omegaDegrees : Double, alphaDegrees : Double, extInches: Double ) : Double {
            val rx = extInches * 0.0254
            val theta = Math.toRadians(thetaDegrees)
            val omega = Math.toRadians(omegaDegrees)
            val alpha = Math.toRadians(alphaDegrees)

            // quadratic coefficient values from excel fit
            val A = 0.21618950
            val B = 0.23259825
            val C = 0.05603656
            val rt = sqrt(A * pow(rx, 2.0) + B * rx + C) // rt is the Center of Mass of the arm (from Excel fit)
            val m = 0.608383140 // mass of the arm in kg (estimated in Excel)
            val jt = m * pow(rt,2.0) // moment of inertia of the arm about the elbow joint
            val b = 0.0 // viscous damping coefficient (assumed 0 for now - not true in reality)

            // all of these are in Volts (will convert to nominal power later)
            val ka = motorResistance * gearRatio * jt / motorKt
            val kv = motorKt/ gearRatio +  motorResistance * gearRatio * b / motorKt
            val kg = cos(theta) * jt * motorResistance * gearRatio * g / motorKt
            val voltage = ka * alpha + kv * omega + kg
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
            val ks = 0.19 // Volts to overcome static friction
            val kk = 0.0 // Volts to overcome kinetic friction
            val omega = Math.toRadians(omegaDegrees)
            val thresholdOmega = 0.01 // rad/s
            val voltage =  sign(power) * (kk + (ks - kk) * exp(-pow(omega / thresholdOmega,2))) // friction model
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
                { ELBOW_MAX_ANGULAR_VELOCITY },
                { ELBOW_MAX_ANGULAR_ACCELERATION },
            )
            motionProfileTimer.reset()
        }
    }


}