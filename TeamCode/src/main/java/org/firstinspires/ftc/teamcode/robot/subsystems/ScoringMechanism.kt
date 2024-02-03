package org.firstinspires.ftc.teamcode.robot.subsystems

import android.util.Log
import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.anchor.util.Timer
import org.firstinspires.ftc.teamcode.util.InverseKinematicsConfig
import org.firstinspires.ftc.teamcode.util.InverseKinematicsConfig.WRIST_ANGLE
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeState
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsystem
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem
import org.joml.Vector2d

/**
 * Overarching scoring mechanism subsystem.
 * Handles the claw, wrist, telescope, and elbow integration.
 */
class ScoringMechanism(
    val leftClaw: LeftClawSubsystem,
    val rightClaw: RightClawSubsystem,
    private val wrist: WristSubsystem,
    private val telescope: TelescopeSubsystem,
    private val elbow: ElbowSubsystem
) : Subsystem() {
    data class KinematicResults(
        val elbowAngle: Double,
        val telescopeExtension: Double,
        val wristAngle: Double
    )

    enum class State {
        CLOSE_INTAKE,
        INTAKE,
        IDLE,
        TRAVEL,
        DEPOSIT,
        CLIMB,
        DROP,
        PREDEPOSIT
    }

    var armState = State.TRAVEL
    private var ikResults = KinematicResults(0.0, 0.0, 0.0)

    var leftClawState = ClawPositions.OPEN
    var rightClawState = ClawPositions.OPEN

    private var currentStateTimer = Timer()
    var pixelHeight = 0.0


    fun switch(state: State) {
        this.currentStateTimer.reset()
        this.armState = state
    }

    override fun init() {

    }

    override fun loop() {

    }

    /**
     * Does inverse kinematics to derive:
     * <ul>
     *     <li>Telescope extension</li>
     *     <li>Elbow angle</li>
     *     <li>Wrist angle</li>
     * </ul>
     */
    fun runKinematics() {
        val retractedTelescopeLength = InverseKinematicsConfig.MINIMUM_EXTENSION

        val goalTrans = pixelHeight * 3.0 + 8.25 // distance along the backdrop from the bottom

        val backdropClawDistance =
            InverseKinematicsConfig.BACKBOARD_OFFSET // distance "off" of the backdrop to target

        val offset = InverseKinematicsConfig.TELESCOPE_OFFSET // offset off of the pivot point

        val backdropAngle =
            Math.toRadians(InverseKinematicsConfig.BACKBOARD_ANGLE) // angle the backdrop is to the ground

        val tPivot = Vector2d(
            -InverseKinematicsConfig.PIVOT_DISTANCE,
            InverseKinematicsConfig.PIVOT_HEIGHT
        ) // relative to the bottom of the backdrop

        val backdropFacing: Vector2d = Vector2d(
            Math.cos(backdropAngle),
            Math.sin(backdropAngle)
        ).normalize() // backdrop normal

        // goal position for the claw to be at
        val goal: Vector2d = backdropFacing.mul(goalTrans, Vector2d())
            .add(
                Math.cos(backdropAngle + Math.PI / 2) * backdropClawDistance,
                Math.sin(backdropAngle + Math.PI / 2) * backdropClawDistance
            )

        val c: Double = goal.distance(tPivot) // initial distance- hypot

        // simple distance- we can assume the angle to be 90 deg on the offset, so we find the correct distance

        // simple distance- we can assume the angle to be 90 deg on the offset, so we find the correct distance
        val telescopeLength = Math.sqrt(c * c - offset * offset)

        val uncompensatedAngle = Math.atan2(goal.y - tPivot.y, goal.x - tPivot.x)
        val finalAngle = -Math.PI / 2 + (uncompensatedAngle + Math.asin(telescopeLength / c))

        // it's really that shrimple
        val elbow = Math.toDegrees(finalAngle)
        val actualElbow = this.elbow.currentAngle


        // it's really that shrimple
        ikResults = KinematicResults(
            180.0 - elbow,
            telescopeLength - retractedTelescopeLength,
            (-(actualElbow - 180) + WRIST_ANGLE) / 2
        )
    }

    fun movementShouldBeComplete(): Boolean {
        Log.d("waitfor", "wrist-" + wrist.movementShouldBeComplete().toString())
        Log.d("waitfor", "telescope-" + telescope.isAtTarget().toString())
        Log.d("waitfor", "elbow-" + elbow.isAtTarget().toString())

        return wrist.movementShouldBeComplete() && elbow.isAtTarget() && telescope.isAtTarget()
    }

    fun setArmState(targetState: State): Component {
        return when (targetState) {
            State.CLOSE_INTAKE -> series(
                SetWristPosition(wrist, WristPosition.CLOSE_INTAKE),
                SetElbowPosition(elbow, ElbowPosition.CLOSE_INTAKE),
                SetTelescopePosition(telescope, TelescopeState.CLOSE_INTAKE),
                instant {
                    armState = targetState
                }
            )

            State.INTAKE -> series(
                SetWristPosition(wrist, WristPosition.EXTENDED_INTAKE),
                SetElbowPosition(elbow, ElbowPosition.EXTENDED_INTAKE),
                SetTelescopePosition(telescope, TelescopeState.EXTENDED_INTAKE),
                instant {
                    armState = targetState
                }
            )

            State.IDLE -> series(
                SetWristPosition(wrist, WristPosition.TRAVEL),
                SetElbowPosition(elbow, ElbowPosition.HOME),
                SetTelescopePosition(telescope, TelescopeState.TRAVEL),
                instant {
                    armState = targetState
                }
            )

            State.TRAVEL -> series(
                SetWristPosition(wrist, WristPosition.TRAVEL),
                SetElbowPosition(elbow, ElbowPosition.TRAVEL),
                SetTelescopePosition(telescope, TelescopeState.TRAVEL),
                instant {
                    armState = targetState
                }
            )

            State.PREDEPOSIT -> series(
                SetWristPosition(wrist, WristPosition.PREDEPOSIT),
                SetElbowPosition(elbow, ElbowPosition.ADJUST),
                SetTelescopePosition(telescope, TelescopeState.ADJUST),

                instant {
                    runKinematics()
                    wrist.depositAngle = ikResults.wristAngle
                    elbow.depositAngle = ikResults.elbowAngle
                    SetTelescopePosition(telescope, ikResults.telescopeExtension)
                },
                instant {
                    armState = targetState
                }
            )

            State.DEPOSIT -> series(
                SetWristPosition(wrist, WristPosition.ADJUST),
                SetElbowPosition(elbow, ElbowPosition.ADJUST),
                SetTelescopePosition(telescope, TelescopeState.ADJUST),

                instant {
                    runKinematics()
                    wrist.depositAngle = ikResults.wristAngle
                    elbow.depositAngle = ikResults.elbowAngle
                },

                SetTelescopePosition(telescope, TelescopeState.ADJUST),
                instant {
                    armState = targetState
                }
            )

            State.CLIMB -> series(
                SetWristPosition(wrist, WristPosition.EXTENDED_INTAKE),
                SetElbowPosition(elbow, ElbowPosition.CLIMB),
                SetTelescopePosition(telescope, TelescopeState.ADJUST),
                SetTelescopePosition(telescope, TelescopeConfig.TELESCOPE_CLIMB),
                instant {
                    armState = targetState
                }
            )

            State.DROP -> series(
                SetTelescopePosition(telescope, TelescopeState.TRAVEL),
                instant {
                    armState = targetState
                }
            )
        }
    }

    override fun end(reason: FinishReason) {

    }

}
