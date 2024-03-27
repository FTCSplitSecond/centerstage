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
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.B_VISCOUS_DAMPING
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_HOME
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_KD
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_KI
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_KP
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_ACCELERATION
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KA_MULTIPLIER
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KG_MULTIPLIER
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KK_VOLTS
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KS_OMEGA_THRESHOLD
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KS_VOLTS
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.KV_MULTIPLIER
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


        // update pid values from config only if they have changed
        if (!(controller.p epsilonEquals  ELBOW_KP && controller.i epsilonEquals  ELBOW_KI && controller.d epsilonEquals  ELBOW_KD ))
            controller.setPID(ELBOW_KP, ELBOW_KI, ELBOW_KD)

        val jt = getMoment(telescope.currentExtensionInches)
        val gravityFeedForward = getGravityFeedforward(motionProfileTarget.x, jt)
        val feedforwardVAPower = getVAFeedForwardPower(motionProfileTarget.v, motionProfileTarget.a, jt)
        val pidPower = controller.calculate(currentAngle, motionProfileTarget.x)
        val basePower = feedforwardVAPower + pidPower
        val frictionPower = getFrictionAdjustment(Math.toRadians(angularV), basePower)
        val totalPower = basePower + frictionPower + gravityFeedForward
        val adjustedPower = adjustForBatteryVoltage(totalPower, hw.voltage())

        if (isEnabled)
            motor power adjustedPower
        else
            motor power 0.0

        lastMotionProfileTarget = motionProfileTarget

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow  : Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle Degree:", targetAngle)
            robot.telemetry.addData("Current Angle Degree", currentAngle)
            robot.telemetry.addData("feedforwardVAPower", feedforwardVAPower)
            robot.telemetry.addData("gravityFeedForward", gravityFeedForward)
            robot.telemetry.addData("pidPower", pidPower)
            robot.telemetry.addData("frictionPower", frictionPower)
            robot.telemetry.addData("totalPower", totalPower)
            robot.telemetry.addData("adjustedPower", adjustedPower)
            robot.telemetry.addData("Angle Error Degree", targetAngle - currentAngle)
            robot.telemetry.addData("Wrist Angle", robot.wrist.angle)
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

            val b = B_VISCOUS_DAMPING // viscous damping coefficient (assumed 0 for now - not true in reality)

            // all of these are in Volts (will convert to nominal power later)
            val ka = KA_MULTIPLIER * motorResistance * gearRatio * jt / motorKt
            val kv = KV_MULTIPLIER * motorKt/ gearRatio +  motorResistance * gearRatio * b / motorKt
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
            val thresholdOmega = KS_OMEGA_THRESHOLD // rad/s
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