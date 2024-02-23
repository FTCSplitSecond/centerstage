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
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_HOME
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_ACCELERATION
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.swerve.utils.clamp
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig.TELESCOPE_MAX
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem


class ElbowSubsystem(private val robot: Robot, private val hw : HardwareManager, val telescope: TelescopeSubsytem) : Subsystem() {

    var isEnabled = true
    var isTelemetryEnabled = false
    private val motor = hw.motor("elbow")


    override fun init() {
        if (robot.opModeType == OpModeType.AUTONOMOUS)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }


    val ELBOW_MOTOR_PPR = 8192.0 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    val DEGREES_PER_REVOLUTION = 360.0  //degrees
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

    /**
     * Angle when the current armState is [ElbowPosition.ADJUST]
     */
    var depositAngle = 0.0

    var position : ElbowPosition = ElbowPosition.Travel
        set(value) {
            targetAngle = value.angle
            field = value
        }

    private val controller = PIDController(ElbowConfig.ELBOW_KP, ElbowConfig.ELBOW_KI, ElbowConfig.ELBOW_KD)


    fun isAtTarget() : Boolean {
        return Math.abs(targetAngle-currentAngle)<PIDTolerance
    }

    var constant = 1.0
    //TODO: What the fuck is this

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

        if (isEnabled) {
            val minExtension = 13.5
            val maxTotalExtension = minExtension + TELESCOPE_MAX
            val currentTotalExtension = minExtension + telescope.currentExtensionInches
            val gravityAdjustment = Math.cos(Math.toRadians(currentAngle)) * (currentTotalExtension/maxTotalExtension) * ElbowConfig.KG
            motor power controller.calculate(currentAngle, motionProfile[motionProfileTimer.seconds()].x) + gravityAdjustment
        } else motor power 0.0

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow  : Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle Degree:", targetAngle)
            robot.telemetry.addData("Current Angle Degree", currentAngle)
            robot.telemetry.addData("Angle Error Degree", targetAngle - currentAngle)
            robot.telemetry.addData("Is At Target", this.isAtTarget())
            robot.telemetry.addData("motor.getCurrrent (mA)", motor.getCurrent() * 1000)
            robot.telemetry.addData("Apple", motionProfile[motionProfileTimer.seconds()].x)
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