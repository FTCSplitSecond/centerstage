package org.firstinspires.ftc.teamcode.robot.opmodes

import LaunchDrone
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.EventTrigger
import dev.turtles.lilypad.impl.FTCGamepad
import dev.turtles.lilypad.module.RoutineModule
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.DropBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawConfig
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class MainTeleOp : AnchorOpMode() {
    lateinit var driver : FTCGamepad
    lateinit var robot : Robot

    override fun prerun() {
        // TODO: Investigate a better way to do this.
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, OpModeType.TELEOP)
        driver = FTCGamepad(gamepad1)
    }

    override fun run() {
        val smec = robot.scoringMechanism
        val leftClaw = robot.leftClaw
        val rightClaw = robot.rightClaw
        val dt = robot.driveBase

        robot.init(this.world)

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
                if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE || smec.armState == ScoringMechanism.State.DEPOSIT) {
                    rightX * 0.5
                } else
                    rightX
            })
        schedule(command)

        val triggerThreshold = 0.2

        val driverRightTrigger = EventTrigger { driver[Button.Trigger.RIGHT] > triggerThreshold }
        val driverLeftTrigger = EventTrigger { driver[Button.Trigger.LEFT] > triggerThreshold }


//        driverLeftTrigger onActivate (MoveToExtendedIntake(robot)) onDeactivate series(
//                    CloseBothClaw(robot.leftclaw, robot.rightClaw),
//                    delay(0.25),
//                    MoveToTravel(robot))
//        driver[Button.Key.CIRCLE] onActivate MoveToCloseIntake(robot)
//
//        driverRightTrigger onActivate MoveToDeposit(robot)
//
//        driver[Button.Key.CROSS] onActivate (
//            series(
//                OpenBothClaw(robot.leftclaw, robot.rightClaw),
//                MoveToPlace(robot),
//                parallel(
//                    CloseBothClaw(robot.leftclaw, robot.rightClaw),
//                    MoveToTravel(robot)
//                )
//            )
//        )
//
//        driver[Button.Key.LEFT_BUMPER] onActivate ToggleLeftClaw(robot.leftclaw)
//        driver[Button.Key.RIGHT_BUMPER] onActivate ToggleRightClaw(robot.rightClaw)
//
//        driver[Button.Key.TRIANGLE] onActivate MoveToTravel(robot)
//        driver[Button.Key.DPAD_UP] onActivate IncreasePixelLevel(robot)
//        driver[Button.Key.DPAD_DOWN] onActivate DecreasePixelLevel(robot)
//        driver[Button.Key.DPAD_LEFT] onActivate LaunchDrone(robot.droneLauncher)


        driver[Button.Key.TRIANGLE] onActivate
                smec.setArmState(ScoringMechanism.State.STACK_INTAKE_CLOSE)

        val routine = RoutineModule { true }
        driver.apply(routine)


//        routine.make {
//            name = "Claw Routine"
//            activation = driver[Button.Key.LEFT_BUMPER]
//            concurrent = false
//
//            action =
//                if (smec.armState == ScoringMechanism.State.INAKE)
//                    instant {
//
//                    } else instant {
//                        println("balls")
//                    }
//        }

        driver[Button.Key.DPAD_DOWN] onActivate instant {
            robot.awayFromDriverStationHeading = robot.driveBase.dt.poseEstimate.heading
        }

        driver[Button.Key.LEFT_BUMPER] onActivate instant {
            robot.leftClaw.position = when (robot.leftClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.CLOSED -> {
                    if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE)
                        ClawPositions.OPEN
                    else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
                        ClawPositions.OPEN
                    else ClawPositions.DROP
                }

                ClawPositions.DROP -> ClawPositions.CLOSED
            }
        }

        driver[Button.Key.RIGHT_BUMPER] onActivate instant {
            robot.rightClaw.position = when (robot.rightClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.DROP -> ClawPositions.CLOSED

                ClawPositions.CLOSED -> {
                    if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE)
                        ClawPositions.OPEN
                    else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
                        ClawPositions.OPEN
                    else ClawPositions.DROP;
                }
            }
        }

        driverLeftTrigger onActivate
                    if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
                        smec.setArmState(ScoringMechanism.State.EXTENDED_INTAKE)
                    else if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE)
                        smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
                    else
                        instant {
                            smec.pixelHeight -= 1.0
                        }

        driverRightTrigger onActivate instant {
            smec.pixelHeight += 1.0
        }

        driver[Button.Key.LEFT_JOSTICK_PRESS] onActivate
            if (smec.armState == ScoringMechanism.State.DEPOSIT)
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            else if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE)
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            else if (smec.armState == ScoringMechanism.State.CLOSE_INTAKE)
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            else
                smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)


        driver[Button.Key.RIGHT_JOYSTICK_PRESS] onActivate instant {
            if (smec.armState == ScoringMechanism.State.DEPOSIT)
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            else
                smec.setArmState(ScoringMechanism.State.DEPOSIT)
        }

        driver[Button.Key.SQUARE] onActivate
                series(
                    CloseBothClaw(robot.leftClaw, robot.rightClaw),
                    smec.setArmState(ScoringMechanism.State.TRAVEL)
                )


        driver[Button.Key.CROSS] onActivate
                series(
                    DropBothClaw(robot.leftClaw, robot.rightClaw),
                    delay(0.25),
                    smec.setArmState(ScoringMechanism.State.TRAVEL)
                )

        driver[Button.Key.START] onActivate
            if (smec.armState == ScoringMechanism.State.CLIMB)
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            else
                smec.setArmState(ScoringMechanism.State.CLIMB)



        driver[Button.Key.DPAD_UP] onActivate LaunchDrone(robot.droneLauncher)


//        driver[Button.Key.SHARE] onActivate instant {
//            LaunchDrone(robot.droneLauncher)
//        }

//        +forever { _, _ ->
//            smec.elbowTesting = driver[Button.Trigger.RIGHT] - driver[Button.Trigger.LEFT]
//        }
//


        +UpdateTelemetry(robot) {
            robot.telemetry.addData("X", robot.driveBase.poseEstimate().x)
            robot.telemetry.addData("Y", robot.driveBase.poseEstimate().y)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngle)
            robot.telemetry.addData("Wrist Angle", robot.wrist.angle)
            robot.telemetry.addData("Telescope Ext", robot.telescope.currentExtensionInches)
            robot.telemetry.addData("left claw", robot.leftClaw.position)
            robot.telemetry.addData("right claw", robot.rightClaw.position)
        }
    }

    override fun end() {
    }
}