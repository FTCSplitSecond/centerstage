package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.electriceel.util.epsilonEquals
import dev.turtles.electriceel.wrapper.HardwareManager
import org.firstinspires.ftc.teamcode.OffseasonBot
import org.firstinspires.ftc.teamcode.common.config.ElbowConfig
import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig
import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig.TELESCOPE_KS
import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig.TELESCOPE_MAX
import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig.TELESCOPE_MIN
import org.firstinspires.ftc.teamcode.common.ktx.adjustForKStatic
import org.firstinspires.ftc.teamcode.common.types.OpModeType
import kotlin.math.PI
import kotlin.math.abs

class TelescopeSubsystem(val robot: OffseasonBot, r: HardwareManager): Subsystem() {

    val leftMotor = r.motor("telescope1")
    val rightMotor = r.motor("telescope2")

    var target = 0.0 // [in]
    var motionProfileTimer = ElapsedTime()
    var previousTarget = target

    var motionProfile = MotionProfileGenerator.generateMotionProfile(
        MotionState(0.0, 0.0),
        MotionState(0.0, 0.0),
        { TelescopeConfig.TELESCOPE_MAX_ACCELERATION },
        { TelescopeConfig.TELESCOPE_MAX_VELOCITY }
    )

    var deltaTimer = ElapsedTime()

    private var x = 0.0
    private var v = 0.0
    private var a = 0.0

    init {
        if (robot.isAuto) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        } else {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        }
    }

    val currentExtensionInches: Double
        get() {
            return getInchesFromTicks(robot.telescopeEncoder.getCounts())
        }

    val inThresh: Boolean
        get() {
            return abs(target - currentExtensionInches) < PID_TOL
        }

    private val controller = PIDController(TelescopeConfig.TELESCOPE_KP, TelescopeConfig.TELESCOPE_KI, TelescopeConfig.TELESCOPE_KD)

    override fun init() {}

    override fun loop() {
        val dt = deltaTimer.seconds()
        deltaTimer.reset()

        val newX = currentExtensionInches
        val newV = (newX - x) / dt
        val newA = (newV - v) / dt

        x = newX
        v = newV
        a = newA

        val currentProfileX = motionProfile[motionProfileTimer.seconds()].x
        val clamped = target.clamp(TELESCOPE_MIN, TELESCOPE_MAX)

        generateMotionProfile(clamped, currentProfileX, v, a)
        val pidPower = controller.calculate(currentExtensionInches, motionProfile[motionProfileTimer.seconds()].x).adjustForKStatic(TELESCOPE_KS)

        if (isEnabled) {
            leftMotor power pidPower
            rightMotor power pidPower
        }
    }

    override fun end(reason: FinishReason) {
        TODO("Not yet implemented")
    }

    private fun generateMotionProfile(target: Double, currentX: Double, currentV: Double, currentA: Double) {
        if (!(previousTarget epsilonEquals target)) {
            previousTarget = target
            motionProfile = MotionProfileGenerator.generateMotionProfile(
                MotionState(currentX, currentV, currentA),
                MotionState(target, 0.0, 0.0),
                { TelescopeConfig.TELESCOPE_MAX_VELOCITY},
                { TelescopeConfig.TELESCOPE_MAX_ACCELERATION },
            )
            motionProfileTimer.reset()
        }
    }

    fun getEncoderTicksFromInches(inches: Double): Double {
        return inches / INCHES_PER_REV * TELESCOPE_MOTOR_PPR
    }

    fun getInchesFromTicks(ticks: Double): Double {
        val d = ticks / TELESCOPE_MOTOR_PPR
        val e = (robot.elbow.currentAngle - ElbowConfig.ELBOW_HOME) / 360.0
        return (d + e) * INCHES_PER_REV
    }

    companion object TelescopeConstants {
        val TELESCOPE_MOTOR_PPR = 145.1 * (18.0 / 19.0)

        val INCHES_PER_REV = 30.0 / 25.4 * PI // [in]
        val PID_TOL = 1.0 // [in]

        var isEnabled = true
    }
}