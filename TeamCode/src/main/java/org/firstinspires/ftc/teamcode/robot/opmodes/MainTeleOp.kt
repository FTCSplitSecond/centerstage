package org.firstinspires.ftc.teamcode.robot.opmodes

import LaunchDrone
import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.EventTrigger
import dev.turtles.lilypad.impl.FTCGamepad
import dev.turtles.lilypad.module.RoutineModule
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.SetLeftClawState
import org.firstinspires.ftc.teamcode.claw.commands.SetRightClawState
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class MainTeleOp : AnchorOpMode() {
    override fun prerun() {
        val driver = FTCGamepad(gamepad1)

        val robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        val smec = robot.scoringMechanism

        val dt = robot.driveBase

        robot.init(this.world)
        robot.elbow.isEnabled = true
        schedule(
            instant {
                robot.droneLauncher.position = DronePositions.HELD
            }
        )

        val command = DriveMecanum(robot.driveBase,
            {
                val leftY = -driver[Button.Joystick.LEFT].y
                leftY.absoluteValue.pow(3) * sign(leftY)
            },
            {
                val leftX = driver[Button.Joystick.LEFT].x
                leftX.absoluteValue.pow(3) * -sign(leftX)
            },
            {
                val rightX = -driver[Button.Joystick.RIGHT].x
                if (smec.armState == ScoringMechanism.State.INTAKE || smec.armState == ScoringMechanism.State.DROP) {
                    rightX * 0.5
                }
                rightX
            })

        schedule(command)

        val triggerThreshold = 0.2

        val driverRightTrigger = EventTrigger { driver[Button.Trigger.RIGHT] > triggerThreshold }
        val driverLeftTrigger = EventTrigger { driver[Button.Trigger.LEFT] > triggerThreshold }


        val routine = RoutineModule { true }
        driver.apply(routine)

        driver[Button.Key.DPAD_DOWN] onActivate instant {
            robot.awayFromDriverStationHeading = robot.driveBase.dt.poseEstimate.heading
        }


        driver[Button.Key.LEFT_BUMPER] onActivate run {
//            val leftClawState = when (smec.leftClaw.position) {
//                ClawPositions.OPEN -> ClawPositions.CLOSED
//                ClawPositions.CLOSED -> {
//                    if (smec.armState == ScoringMechanism.State.INTAKE)
//                        ClawPositions.OPEN
//                    else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
//                        ClawPositions.OPEN
//                    else ClawPositions.DROP
//                }
//
//                ClawPositions.DROP -> ClawPositions.CLOSED
//            }
//            val leftClawState = ClawPositions.OPEN
            SetLeftClawState(smec.leftClaw, ClawPositions.OPEN)
        }

        driver[Button.Key.RIGHT_BUMPER] onActivate run {
            val rightClawState = when (smec.rightClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.CLOSED -> {
                    if (smec.armState == ScoringMechanism.State.INTAKE)
                        ClawPositions.OPEN
                    else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
                        ClawPositions.OPEN
                    else ClawPositions.DROP
                }

                ClawPositions.DROP -> ClawPositions.CLOSED
            }
            SetRightClawState(smec.rightClaw, rightClawState)
        }

        driverLeftTrigger onActivate run {
            if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
                smec.setArmState(ScoringMechanism.State.INTAKE)
            else if (smec.armState == ScoringMechanism.State.INTAKE)
                smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
            else
                instant {
                    smec.pixelHeight -= 1.0
                }
        }

        driverRightTrigger onActivate instant {
            smec.pixelHeight += 1.0
        }

        driver[Button.Key.LEFT_JOSTICK_PRESS] onActivate run {
            series(
                when (smec.armState) {
                    ScoringMechanism.State.TRAVEL, ScoringMechanism.State.INTAKE, ScoringMechanism.State.CLOSE_INTAKE -> smec.setArmState(
                        ScoringMechanism.State.TRAVEL
                    )

                    else -> smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
                },
                OpenBothClaw(smec.leftClaw, smec.rightClaw)
            )
        }

        driver[Button.Key.CROSS] onActivate
                series(
                    parallel(
                        SetRightClawState(smec.rightClaw, ClawPositions.DROP),
                        SetLeftClawState(smec.leftClaw, ClawPositions.DROP)
                    ),
                    delay(0.25),
                    smec.setArmState(ScoringMechanism.State.DROP),
                    smec.setArmState(ScoringMechanism.State.TRAVEL)
                )

        driver[Button.Key.START] onActivate run {
            if (smec.armState == ScoringMechanism.State.CLIMB) {
                parallel(
                    smec.setArmState(ScoringMechanism.State.DROP),
                    series(
                        delay(0.75),
                        smec.setArmState(ScoringMechanism.State.TRAVEL),
                    )
                )
            } else {
                smec.setArmState(ScoringMechanism.State.CLIMB)
            }
        }

        driver[Button.Key.DPAD_UP] onActivate LaunchDrone(robot.droneLauncher)




        +UpdateTelemetry(robot) {
            robot.telemetry.addData("X", robot.driveBase.poseEstimate().x)
            robot.telemetry.addData("Y", robot.driveBase.poseEstimate().y)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngle)
            robot.telemetry.addData("Wrist Angle", robot.wrist.angle)
            robot.telemetry.addData("Telescope Ext", robot.telescope.currentExtensionInches)
            robot.telemetry.addData("left claw", robot.leftClaw.position)
            robot.telemetry.addData("right claw", robot.rightClaw.position)

            robot.telemetry.update()
        }
    }

    override fun run() {

    }
}