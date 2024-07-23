package org.firstinspires.ftc.teamcode.subsystem

import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import org.firstinspires.ftc.teamcode.common.config.IVKConfig
import org.firstinspires.ftc.teamcode.common.config.IVKConfig.WRIST_ANGLE
import org.firstinspires.ftc.teamcode.common.config.WristConfig
import org.firstinspires.ftc.teamcode.subsystem.elbow.ElbowSubsystem
import org.firstinspires.ftc.teamcode.subsystem.telescope.TelescopeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.wrist.WristSubsystem
import org.joml.Vector2d
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class DepositSubsystem(
    private val claw: ClawSubsystem,
    private val wrist: WristSubsystem,
    private val telescope: TelescopeSubsystem,
    private val elbow: ElbowSubsystem
) {
    data class KinematicResults(
        val telescopeExtension: Double,
        val elbowAngle: Double,
        val wristAngle: Double,
        val depositCoRX: Double
    )

    enum class State {
        CLOSED_INTAKE,
        EXTENDED_INTAKE,
        TRAVEL,
        DEPOSIT,
        CLIMB,
        STACK_INTAKE,
        STACK_INTAKE_CLOSED
    }

    var armState = State.TRAVEL
        private set

    var depositPixelLevel = 0.0
        private set

    /**
     * Does the inverse kinematics to derive
     * - Telescope extension
     * - Elbow angle
     * - Wrist angle
     */
    fun runKinematics(pixelHeight: Double): KinematicResults {
        val retractedTelescopeLength = IVKConfig.MINIMUM_EXTENSION

        val goalTrans = pixelHeight * 3.0 + 8.25

        val backDropClawDist = IVKConfig.BACKBOARD_OFFSET
        val offset = IVKConfig.TELESCOPE_OFFSET

        val backdropAngle = Math.toRadians(IVKConfig.BACKBOARD_ANGLE)

        val tPivot = Vector2d(
            -IVKConfig.PIVOT_DISTANCE,
            IVKConfig.PIVOT_HEIGHT
        )

        val bdFacing = Vector2d(
            cos(backdropAngle),
            sin(backdropAngle)
        ).normalize()

        val goal = bdFacing.mul(goalTrans, Vector2d())
            .add(
                cos(backdropAngle + PI / 2.0) * backDropClawDist,
                sin(backdropAngle + PI / 2.0) * backDropClawDist
            )

        val c = goal.distance(tPivot)
        val telescopeLength = sqrt(c * c - offset * offset)

        val uncompensatedAngle = atan2(goal.y - tPivot.y, goal.x - tPivot.x)
        val finalAngle = -PI / 2 + (uncompensatedAngle + asin(telescopeLength / c))

        val elbow = Math.toDegrees(finalAngle)
        val depositCoRX = -5.0 - telescopeLength * cos(finalAngle)

        val kinResults = KinematicResults(
            180.0 - elbow,
            telescopeLength - retractedTelescopeLength,
            (elbow + WRIST_ANGLE) / 2 + WristConfig.WRIST_OFFSET,
            depositCoRX
        )

        return kinResults
    }

    fun getDepositXCenterOfRotation(): Double {
        val ikResults = runKinematics(depositPixelLevel)
        return ikResults.depositCoRX
    }

    fun setPixelLevel(pixelLevel: Double): Component {
        depositPixelLevel = pixelLevel

        return when (armState) {
            State.DEPOSIT -> {
                val ikResults = runKinematics(depositPixelLevel)
                parallel()
            }
            else -> instant {}
        }
    }
}