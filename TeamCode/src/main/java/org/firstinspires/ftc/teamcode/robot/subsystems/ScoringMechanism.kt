package org.firstinspires.ftc.teamcode.robot.subsystems

import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import org.firstinspires.ftc.teamcode.util.InverseKinematicsConfig
import org.firstinspires.ftc.teamcode.util.InverseKinematicsConfig.WRIST_ANGLE
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
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
                       private val elbow: ElbowSubsystem) {
    data class KinematicResults(val elbowAngle : Double, val telescopeExtension : Double, val wristAngle : Double)
    enum class State {
        CLOSE_INTAKE,
        EXTENDED_INTAKE,
        TRAVEL,
        DEPOSIT,
        CLIMB,
        STACK_INTAKE,
        STACK_INTAKE_CLOSE
    }

    var armState = State.TRAVEL
        private set;
    var pixelHeight = 0.0

    /**
     * Does inverse kinematics to derive:
     * <ul>
     *     <li>Telescope extension</li>
     *     <li>Elbow angle</li>
     *     <li>Wrist angle</li>
     * </ul>
     */
    private fun runKinematics(pixelHeight : Double) : KinematicResults {

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


        // it's really that shrimple
        return KinematicResults(
                180.0 - elbow,
                telescopeLength - retractedTelescopeLength,
            (elbow + WRIST_ANGLE)/2
        )
    }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    fun setArmState(newState: State): Component {
        val updateState = instant {
            armState = newState
        }
        return series(
            when (newState) {
                State.CLOSE_INTAKE -> parallel(
                    //TODO redo series
                    SetWristPosition(wrist, WristPosition.CloseIntake),
                    SetElbowPosition(elbow, ElbowPosition.CloseIntake),
                    SetTelescopePosition(telescope, TelescopePosition.CloseIntake),
                )
=======
=======
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
    fun setArmState(newState : State) : Component {
        val updateState = instant { armState = newState}
        return series(when(newState){
            State.CLOSE_INTAKE -> parallel(
                //TODO redo series
                SetWristPosition(wrist, WristPosition.CloseIntake),
                SetElbowPosition(elbow, ElbowPosition.CloseIntake),
                SetTelescopePosition(telescope, TelescopePosition.CloseIntake),
            )
<<<<<<< HEAD
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
=======
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)

            State.EXTENDED_INTAKE -> parallel(
                SetWristPosition(wrist, WristPosition.ExtendedIntake),
                SetElbowPosition(elbow, ElbowPosition.ExtendedIntake),
                SetTelescopePosition(telescope, TelescopePosition.ExtendedIntake),
            )

<<<<<<< HEAD
<<<<<<< HEAD
                State.TRAVEL -> when (armState) {
                    State.DEPOSIT ->
=======
    fun setArmState(newState : State) : Component {
        val updateState = instant { armState = newState}
        return when(newState){
            State.CLOSE_INTAKE -> parallel(
                //TODO redo series
                SetWristPosition(wrist, WristPosition.CloseIntake),
                SetElbowPosition(elbow, ElbowPosition.CloseIntake),
                SetTelescopePosition(telescope, TelescopePosition.CloseIntake),
                updateState
            )

            State.EXTENDED_INTAKE -> parallel(
                SetWristPosition(wrist, WristPosition.ExtendedIntake),
                SetElbowPosition(elbow, ElbowPosition.ExtendedIntake),
                SetTelescopePosition(telescope, TelescopePosition.ExtendedIntake),
                updateState
            )

            State.TRAVEL -> when(armState) {
                State.DEPOSIT ->
                    series(
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
                        parallel(
                            SetElbowPosition(elbow, ElbowPosition.Travel),
                            SetWristPosition(wrist, WristPosition.Travel)
                        ),
                        updateState

                    )
                State.CLIMB ->
                    parallel(
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
>>>>>>> parent of 4cb1952 (Fixed TERRIBLE HORRIBLE AWFUL wrist issue)
                        series(
                            SetTelescopePosition(telescope, TelescopePosition.Travel),
                            parallel(
                                SetElbowPosition(elbow, ElbowPosition.Travel),
                                SetWristPosition(wrist, WristPosition.Travel)
                            ),
                        )

                    State.CLIMB ->
                        parallel(
                            SetTelescopePosition(telescope, TelescopePosition.Travel),
                            series(
                                delay(0.25),
                                SetElbowPosition(elbow, ElbowPosition.Travel),
                                SetWristPosition(wrist, WristPosition.Travel)
                            ),
                        )

                    else ->
                        parallel(
                            SetWristPosition(wrist, WristPosition.Travel),
                            SetElbowPosition(elbow, ElbowPosition.Travel),
                            SetTelescopePosition(telescope, TelescopePosition.Travel),
                        )
                }

                State.DEPOSIT -> {
                    val ikResults = runKinematics(pixelHeight)
=======
            State.TRAVEL -> when(armState) {
                State.DEPOSIT ->
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
=======
            State.TRAVEL -> when(armState) {
                State.DEPOSIT ->
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
                    series(
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
                        parallel(
                            SetElbowPosition(elbow, ElbowPosition.Travel),
                            SetWristPosition(wrist, WristPosition.Travel)
                        ),
                        updateState
                    )
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
                State.CLIMB ->
                    parallel(
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
                        series(
                            delay(0.25),
                            SetElbowPosition(elbow, ElbowPosition.Travel),
                            SetWristPosition(wrist, WristPosition.Travel)
                        ),
                    )
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
                else ->
                    parallel(
                        SetWristPosition(wrist, WristPosition.Travel),
                        SetElbowPosition(elbow, ElbowPosition.Travel),
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
<<<<<<< HEAD
                        updateState
                    )
>>>>>>> parent of 4cb1952 (Fixed TERRIBLE HORRIBLE AWFUL wrist issue)
=======
                    )
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
                }

            State.DEPOSIT -> {
                val ikResults = runKinematics(pixelHeight)
                series(
                    SetElbowPosition(elbow, ElbowPosition.Adjust(ikResults.elbowAngle)),
                    parallel(
                        SetTelescopePosition(telescope, TelescopePosition.Adjust(ikResults.telescopeExtension)),
                        SetWristPosition(wrist, WristPosition.Adjust(ikResults.wristAngle))
                    ),
                    updateState
                )
            }

<<<<<<< HEAD
<<<<<<< HEAD
                State.STACK_INTAKE_CLOSE -> series(
                    SetElbowPosition(elbow, ElbowPosition.Travel),
                    parallel(
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
                        SetWristPosition(wrist, WristPosition.Travel)
                    ),
                )

                State.CLIMB -> series(
                    SetElbowPosition(elbow, ElbowPosition.Travel),
                    parallel(
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
                        SetWristPosition(wrist, WristPosition.Travel)
                    ),
                )
            },
            updateState
        )
=======
            State.STACK_INTAKE -> series(
                SetElbowPosition(elbow, ElbowPosition.Travel),
                parallel(
                    SetTelescopePosition(telescope, TelescopePosition.Travel),
                    SetWristPosition(wrist, WristPosition.Travel)
                ),
                updateState
            )

=======
            State.STACK_INTAKE -> series(
                SetElbowPosition(elbow, ElbowPosition.Travel),
                parallel(
                    SetTelescopePosition(telescope, TelescopePosition.Travel),
                    SetWristPosition(wrist, WristPosition.Travel)
                ),
            )

>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
=======
                State.CLIMB ->
                    parallel(
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
                        series(
                            delay(0.25),
                            SetElbowPosition(elbow, ElbowPosition.Travel),
                            SetWristPosition(wrist, WristPosition.Travel)
                        ),
                    )
                else ->
                    parallel(
                        SetWristPosition(wrist, WristPosition.Travel),
                        SetElbowPosition(elbow, ElbowPosition.Travel),
                        SetTelescopePosition(telescope, TelescopePosition.Travel),
                    )
                }

            State.DEPOSIT -> {
                val ikResults = runKinematics(pixelHeight)
                series(
                    SetElbowPosition(elbow, ElbowPosition.Adjust(ikResults.elbowAngle)),
                    parallel(
                        SetTelescopePosition(telescope, TelescopePosition.Adjust(ikResults.telescopeExtension)),
                        SetWristPosition(wrist, WristPosition.Adjust(ikResults.wristAngle))
                    ),
                )
            }

            State.STACK_INTAKE -> series(
                SetElbowPosition(elbow, ElbowPosition.Travel),
                parallel(
                    SetTelescopePosition(telescope, TelescopePosition.Travel),
                    SetWristPosition(wrist, WristPosition.Travel)
                ),
            )

>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
            State.STACK_INTAKE_CLOSE -> series(
                SetElbowPosition(elbow, ElbowPosition.Travel),
                parallel(
                    SetTelescopePosition(telescope, TelescopePosition.Travel),
                    SetWristPosition(wrist, WristPosition.Travel)
                ),
<<<<<<< HEAD
<<<<<<< HEAD
                updateState
=======
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
=======
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
            )

            State.CLIMB -> series(
                SetElbowPosition(elbow, ElbowPosition.Travel),
                parallel(
                    SetTelescopePosition(telescope, TelescopePosition.Travel),
                    SetWristPosition(wrist, WristPosition.Travel)
                ),
<<<<<<< HEAD
<<<<<<< HEAD
                updateState
            )
        }
>>>>>>> parent of 4cb1952 (Fixed TERRIBLE HORRIBLE AWFUL wrist issue)
=======
            )
        },
            updateState)
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
=======
            )
        },
            updateState)
>>>>>>> parent of efd0440 (Fixed claws and ported most instants over to components)
    }
}
