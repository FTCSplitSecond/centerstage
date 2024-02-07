package org.firstinspires.ftc.teamcode.robot.subsystems

import android.util.Log
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import dev.turtles.anchor.util.Timer
import org.firstinspires.ftc.teamcode.util.InverseKinematicsConfig
import org.firstinspires.ftc.teamcode.util.InverseKinematicsConfig.WRIST_ANGLE
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristConfig.WRIST_MOVE_OFFSET
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem
import org.joml.Vector2d

/**
 * Overarching scoring mechanism subsystem.
 * Handles the claw, wrist, telescope, and elbow integration.
 */
class ScoringMechanism(private val leftClaw: LeftClawSubsystem,
                       private val rightClaw: RightClawSubsystem,
                       private val wrist: WristSubsystem,
                       private val telescope: TelescopeSubsytem,
                       private val elbow: ElbowSubsystem) : Subsystem() {
    data class KinematicResults(val elbowAngle : Double, val telescopeExtension : Double, val wristAngle : Double, val moveWristAngle : Double)
    enum class State {
        CLOSE_INTAKE,
        INTAKE,
        IDLE,
        TRAVEL,
        DEPOSIT,
        CLIMB,
        DROP,
        MOVEDEPOSIT,
        STACK_INTAKE,
        STACK_INTAKE_CLOSE
    }

    var state = State.TRAVEL
    private var ikResults = KinematicResults(0.0, 0.0, 0.0, 0.0)

    var leftClawState = ClawPositions.OPEN
    var rightClawState = ClawPositions.OPEN

    private var currentStateTimer = Timer()
    var pixelHeight = 0.0


    fun switch(state: State) {
        this.currentStateTimer.reset()
        this.state = state
    }

    override fun init() {

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

        val backdropClawDistance = InverseKinematicsConfig.BACKBOARD_OFFSET // distance "off" of the backdrop to target

        val offset = InverseKinematicsConfig.TELESCOPE_OFFSET // offset off of the pivot point

        val backdropAngle = Math.toRadians(InverseKinematicsConfig.BACKBOARD_ANGLE) // angle the backdrop is to the ground

        val tPivot = Vector2d(-InverseKinematicsConfig.PIVOT_DISTANCE, InverseKinematicsConfig.PIVOT_HEIGHT) // relative to the bottom of the backdrop

        val backdropFacing: Vector2d = Vector2d(Math.cos(backdropAngle), Math.sin(backdropAngle)).normalize() // backdrop normal

        // goal position for the claw to be at
        val goal: Vector2d = backdropFacing.mul(goalTrans, Vector2d())
                .add(Math.cos(backdropAngle + Math.PI / 2) * backdropClawDistance, Math.sin(backdropAngle + Math.PI / 2) * backdropClawDistance)

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
            (-(actualElbow - 180) + WRIST_ANGLE)/2,
            (-(actualElbow - 180) + WRIST_ANGLE)/2 - WRIST_MOVE_OFFSET
        )
    }
    fun movementShouldBeComplete() : Boolean {
        Log.d("waitfor", "wrist-"+wrist.movementShouldBeComplete().toString())
        Log.d("waitfor", "telescope-"+telescope.isAtTarget().toString())
        Log.d("waitfor", "elbow-"+elbow.isAtTarget().toString())

        return wrist.movementShouldBeComplete() && elbow.isAtTarget() && telescope.isAtTarget()
    }
    override fun loop() {

        // This is a crude state machine.
        // A state machine library, such as the WIP one in Nautilus could be used
        when (state) {

            State.CLOSE_INTAKE -> {
                wrist.position = WristPosition.CLOSE_INTAKE
                elbow.position = ElbowPosition.CLOSE_INTAKE
                telescope.position = TelescopePosition.CLOSE_INTAKE
            }
            State.INTAKE -> {
                wrist.position = WristPosition.EXTENDED_INTAKE
                elbow.position = ElbowPosition.EXTENDED_INTAKE
                telescope.position = TelescopePosition.EXTENDED_INTAKE
            }
            State.IDLE -> {
                wrist.position = WristPosition.TRAVEL
                elbow.position = ElbowPosition.HOME
                telescope.position = TelescopePosition.TRAVEL
            }
            State.TRAVEL -> {
                wrist.position = WristPosition.TRAVEL
                elbow.position = ElbowPosition.TRAVEL
                telescope.position = TelescopePosition.TRAVEL
            }
            State.MOVEDEPOSIT -> {
                wrist.position = WristPosition.ADJUST
                elbow.position = ElbowPosition.ADJUST
                telescope.position = TelescopePosition.ADJUST

                runKinematics()

                wrist.depositAngle = ikResults.moveWristAngle
                elbow.depositAngle = ikResults.elbowAngle
                telescope.targetExtenstionInches = ikResults.telescopeExtension
            }
            State.DEPOSIT -> {
                wrist.position = WristPosition.ADJUST
                elbow.position = ElbowPosition.ADJUST
                telescope.position = TelescopePosition.ADJUST

                runKinematics()

                wrist.depositAngle = ikResults.wristAngle
                elbow.depositAngle = ikResults.elbowAngle
                telescope.targetExtenstionInches = ikResults.telescopeExtension
            }

            State.CLIMB -> {
                wrist.position = WristPosition.EXTENDED_INTAKE
                elbow.position = ElbowPosition.CLIMB
                telescope.position = TelescopePosition.ADJUST
                telescope.targetExtenstionInches = TelescopeConfig.TELESCOPE_CLIMB
            }

            State.DROP -> {
                telescope.position = TelescopePosition.TRAVEL
            }
            State.STACK_INTAKE -> {
                telescope.position = TelescopePosition.CLOSE_INTAKE
                elbow.position = ElbowPosition.STACK_INTAKE
                wrist.position = WristPosition.CLOSE_INTAKE
            }

            State.STACK_INTAKE_CLOSE -> {
                telescope.position = TelescopePosition.EXTENDED_INTAKE
                elbow.position = ElbowPosition.STACK_INTAKE_CLOSE
                wrist.position = WristPosition.EXTENDED_INTAKE
            }
        }

        leftClaw.position = leftClawState
        rightClaw.position = rightClawState
    }

    override fun end(reason: FinishReason) {

    }

}
