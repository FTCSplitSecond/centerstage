package org.firstinspires.ftc.teamcode.subsystem.elbow

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.util.epsilonEquals
import org.firstinspires.ftc.teamcode.SplitSecondBot
import org.firstinspires.ftc.teamcode.common.config.ElbowConfig
import org.firstinspires.ftc.teamcode.common.ktx.adjustForKStatic
import kotlin.math.abs
import kotlin.math.cos

class ElbowSubsystem(val robot: SplitSecondBot): Subsystem() {

    val telescope = robot.telescope
    val motor = robot.hw.motor("elbow")

    override fun init() {
        if (robot.isAuto)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    var target = 0.0
        private set

    var previousTarget = target

    val motionProfilerTimer = ElapsedTime()

    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(currentAngle, 0.0, 0.0),
        MotionState(target, 0.0, 0.0),
        { ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY },
        { ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY }
    )

    var x: Double = currentAngle
    var v: Double = 0.0
    var a: Double = 0.0

    private val deltaTimer = ElapsedTime()

    val currentAngle: Double
        get()  {
            return robot.elbowEncoder.getCounts()
        }

    var position: ElbowPositions = ElbowPositions.Travel
        set(value) {
            target = value.angle
            field = value
        }

    val inThresh: Boolean
        get() {
            return abs(target - currentAngle) < PID_TOL
        }

    private val controller = PIDController(ElbowConfig.ELBOW_KP, ElbowConfig.ELBOW_KI, ElbowConfig.ELBOW_KD)

    override fun loop() {
        val dt = deltaTimer.seconds()
        deltaTimer.reset()

        val newX = currentAngle
        val newV = (newX - x) / dt
        val newA = (newV - v) / dt

        x = newX
        v = newV
        a = newA

        val currentProfileX = motionProfile[motionProfilerTimer.seconds()].x

        val clampedTarget = target.clamp(
            ElbowConfig.ELBOW_MIN,
            ElbowConfig.ELBOW_MAX
        )

        if (isEnabled) {
            val gravityAdjust = cos(Math.toRadians(currentAngle)) * ElbowConfig.KG
            val pidPower = controller.calculate(currentAngle, motionProfile[motionProfilerTimer.seconds()].x).adjustForKStatic(ElbowConfig.KS)
            motor power pidPower + gravityAdjust
        } else motor power 0.0
    }

    override fun end(reason: FinishReason) {}

    private fun generateMotionProfile(target: Double, currentX: Double, currentV: Double, currentA: Double) {
        if (!(previousTarget epsilonEquals target)) {
            previousTarget = target
            motionProfile = MotionProfileGenerator.generateMotionProfile(
                MotionState(currentX, currentV, currentA),
                MotionState(target, 0.0, 0.0),
                { ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY },
                { ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY }
            )
            motionProfilerTimer.reset()
        }
    }

    companion object {
        val ELBOW_MOTOR_PPR = 751.8
        val DEGREES_PER_REV = 360 * (14.0 / 53.0) // [deg]
        val PID_TOL = 5.0 // [deg]

        var isEnabled = true
    }
}