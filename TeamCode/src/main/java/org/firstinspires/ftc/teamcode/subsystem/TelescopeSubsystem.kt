package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import org.firstinspires.ftc.teamcode.OffseasonBot
import org.firstinspires.ftc.teamcode.common.config.TelescopeConfig
import org.firstinspires.ftc.teamcode.common.types.OpModeType
import kotlin.math.PI

class TelescopeSubsystem(robot: OffseasonBot): Subsystem() {

    val hardwareManager = robot.hw

    val leftMotor = hardwareManager.motor("telescope1")
    val rightMotor = hardwareManager.motor("telescope2")

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
        if (robot.opModeType == OpModeType.AUTO) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        } else {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        }
    }

    override fun init() {
        TODO("Not yet implemented")
    }

    override fun loop() {
        TODO("Not yet implemented")
    }

    override fun end(reason: FinishReason) {
        TODO("Not yet implemented")
    }

    fun getEncoderTicksFromInches(inches: Double): Double {
        return inches / INCHES_PER_REV * TELESCOPE_MOTOR_PPR
    }

    fun getInchesFromTicks(ticks: Double): Double {
        val d = ticks / TELESCOPE_MOTOR_PPR
//        val e=

        return 0.0
    }

    companion object TelescopeConstants {
        val TELESCOPE_MOTOR_PPR = 145.1 * (18.0 / 19.0)

        val INCHES_PER_REV = 30.0 / 25.4 * PI // [in]
        val PID_TOLERANCE = 1.0 // [in]
    }
}