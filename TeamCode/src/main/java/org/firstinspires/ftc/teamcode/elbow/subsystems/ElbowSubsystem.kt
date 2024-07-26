package org.firstinspires.ftc.teamcode.elbow.subsystems

import android.util.Log
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
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.HOMING_ANGULARVELOCITY_THRESHOLD
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig.HOMING_POWER
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.util.adjustPowerForKStatic
import org.firstinspires.ftc.teamcode.swerve.utils.clamp
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig.HOMING_VELOCITY_THRESHOLD
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsystem
import kotlin.math.abs


class ElbowSubsystem(private val robot: Robot, private val hw : HardwareManager, val telescope: TelescopeSubsystem) : Subsystem() {

    var isEnabled = true
    var isTelemetryEnabled = false
    var isHoming = false
    private var hasHomingPowerBeenSet = false
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
    var deltaTimer = ElapsedTime()
    private fun getEncoderTicksFromAngle(angle : Double) : Double {
        return angle/ DEGREES_PER_REVOLUTION * ELBOW_MOTOR_PPR // ticks
    }
    private fun getAngleFromEncoderTicks(encoderTicks : Double) : Double {
        return  encoderTicks/ ELBOW_MOTOR_PPR * DEGREES_PER_REVOLUTION + ELBOW_HOME // degrees
    }

    var currentAngle : Double = getAngleFromEncoderTicks(motor.encoder.getCounts())
        private set

    private var angularX : Double = currentAngle
    private var angularV : Double = 0.0
    private var angularA : Double = 0.0
    private var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentAngle, 0.0, 0.0),
        MotionState(targetAngle, 0.0, 0.0),
        { ELBOW_MAX_ANGULAR_VELOCITY },
        { ELBOW_MAX_ANGULAR_ACCELERATION },
    )
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
        currentAngle = getAngleFromEncoderTicks(motor.encoder.getCounts())
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
        val gravityAdjustment = Math.cos(Math.toRadians(currentAngle))  * ElbowConfig.KG
        val pidPower = controller.calculate(currentAngle, motionProfile[motionProfileTimer.seconds()].x).adjustPowerForKStatic(ElbowConfig.KS)
        motor power pidPower + gravityAdjustment

        if (isEnabled) {
            if(isHoming){
                Log.d("elbow","homing angular velocity $angularV, abs threshold is $HOMING_ANGULARVELOCITY_THRESHOLD")
                if(!hasHomingPowerBeenSet) {
                    Log.d("elbow","homing starting")
                    Log.d("elbow","homing power set to $HOMING_POWER")
                    motor power HOMING_POWER
                    hasHomingPowerBeenSet = true
                } else if(abs(angularV) < HOMING_ANGULARVELOCITY_THRESHOLD) {
                    Log.d("elbow","homing completed")
                    motor power 0.0
                    // we are at the end stop, reset the encoders, re-initialize the current/target
                    // and end homing sequence
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    currentAngle = ELBOW_HOME
                    position = ElbowPosition.Travel
                    isHoming = false
                    hasHomingPowerBeenSet = false
                } else {
                    motor power HOMING_POWER
                }
            }

        } else motor power 0.0

        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Elbow  : Telemetry Enabled")
            robot.telemetry.addData("IsEnabled:", isEnabled)
            robot.telemetry.addData("Target Angle Degree:", targetAngle)
            robot.telemetry.addData("Current Angle Degree", currentAngle)
            robot.telemetry.addData("Angle Error Degree", targetAngle - currentAngle)
            robot.telemetry.addData("Is At Target", this.isAtTarget())
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