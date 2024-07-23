package org.firstinspires.ftc.teamcode.subsystem

import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.idler
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import org.firstinspires.ftc.teamcode.common.config.IVKConfig
import org.firstinspires.ftc.teamcode.common.config.IVKConfig.WRIST_ANGLE
import org.firstinspires.ftc.teamcode.common.config.WristConfig
import org.firstinspires.ftc.teamcode.component.elbow.SetElbowPosition
import org.firstinspires.ftc.teamcode.component.telescope.SetTelescopePosition
import org.firstinspires.ftc.teamcode.component.wrist.SetWristPosition
import org.firstinspires.ftc.teamcode.subsystem.elbow.ElbowPositions
import org.firstinspires.ftc.teamcode.subsystem.elbow.ElbowSubsystem
import org.firstinspires.ftc.teamcode.subsystem.telescope.TelescopePositions
import org.firstinspires.ftc.teamcode.subsystem.telescope.TelescopeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.wrist.WristPositions
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

    fun setArmState(newState: State): Component {
        val updateState = instant {
            armState = newState
        }

        return series(
            when (newState) {
                State.CLOSED_INTAKE -> parallel(
                    SetWristPosition(wrist, WristPositions.CloseIntake),
                    SetElbowPosition(elbow, ElbowPositions.CloseIntake),
                    SetTelescopePosition(telescope, TelescopePositions.CloseIntake)
                )
                State.EXTENDED_INTAKE -> parallel(
                    SetWristPosition(wrist, WristPositions.ExtendedIntake),
                    SetElbowPosition(elbow, ElbowPositions.ExtendedIntake),
                    SetTelescopePosition(telescope, TelescopePositions.ExtendedIntake)
                )
                State.TRAVEL -> when (armState) {
                    State.DEPOSIT -> series(
                        SetTelescopePosition(telescope, TelescopePositions.Travel),
                        parallel(
                            SetElbowPosition(elbow, ElbowPositions.Travel),
                            series(
                                idler { _, elapsedTime -> elbow.currentAngle < 100.0 || elapsedTime > 0.25 },
                                SetWristPosition(wrist, WristPositions.Travel)
                            )
                        )
                    )
                    State.CLIMB -> parallel(
                        SetTelescopePosition(telescope, TelescopePositions.Travel),
                        series(
                            delay(0.25),
                            SetElbowPosition(elbow, ElbowPositions.Travel),
                            SetWristPosition(wrist, WristPositions.Travel)
                        )
                    )
                    else -> parallel(
                        SetTelescopePosition(telescope, TelescopePositions.Travel),
                        SetElbowPosition(elbow, ElbowPositions.Travel),
                        SetWristPosition(wrist, WristPositions.Travel)
                    )
                }
                State.DEPOSIT -> {
                    val ivkResults = runKinematics(depositPixelLevel)

                    series(
                        parallel(
                            SetElbowPosition(elbow, ElbowPositions.Adjust(ivkResults.elbowAngle)),
                            series(
                                delay(0.25),
                                SetWristPosition(wrist, WristPositions.Adjust(ivkResults.wristAngle))
                            ),
                            SetTelescopePosition(telescope, TelescopePositions.Adjust(ivkResults.telescopeExtension)))
                    )
                }
                State.CLIMB -> series(
                    parallel(
                        SetElbowPosition(elbow, ElbowPositions.Climb),
                        SetWristPosition(wrist, WristPositions.Travel)
                    ),
                    SetTelescopePosition(telescope, TelescopePositions.Climb)
                )
                State.STACK_INTAKE -> series(
                    SetElbowPosition(elbow, ElbowPositions.Travel),
                    parallel(
                        SetTelescopePosition(telescope, TelescopePositions.Travel),
                        SetWristPosition(wrist, WristPositions.Travel)
                    )
                )
                State.STACK_INTAKE_CLOSED -> series(
                    parallel(
                        SetElbowPosition(elbow, ElbowPositions.Climb),
                        SetWristPosition(wrist, WristPositions.Travel)
                    ),
                    SetTelescopePosition(telescope, TelescopePositions.Travel)
                )
            }, updateState
        )
    }

    fun getDepositXCenterOfRotation(): Double {
        val ikResults = runKinematics(depositPixelLevel)
        return ikResults.depositCoRX
    }

    fun setPixelLevel(pixelLevel: Double): Component {
        depositPixelLevel = pixelLevel

        return when (armState) {
            State.DEPOSIT -> {
                val ivkResults = runKinematics(depositPixelLevel)
                parallel(
                    SetTelescopePosition(telescope, TelescopePositions.Adjust(ivkResults.telescopeExtension)),
                    SetElbowPosition(elbow, ElbowPositions.Adjust(ivkResults.elbowAngle)),
                    SetWristPosition(wrist, WristPositions.Adjust(ivkResults.wristAngle))
                )
            }
            else -> instant {}
        }
    }
}