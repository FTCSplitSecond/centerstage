package org.firstinspires.ftc.teamcode.robot.opmodes

import LaunchDrone
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.SequentialParent
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.forever
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.electriceel.util.AngleUnit
import dev.turtles.electriceel.util.epsilonEquals
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.EventTrigger
import dev.turtles.lilypad.impl.FTCGamepad
import dev.turtles.lilypad.module.RoutineModule
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import kotlin.math.PI
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
            { -driver[Button.Joystick.RIGHT].x })

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

        val routine = RoutineModule { true }
        driver.apply(routine)


//        routine.make {
//            name = "Claw Routine"
//            activation = driver[Button.Key.LEFT_BUMPER]
//            concurrent = false
//
//            action =
//                if (smec.state == ScoringMechanism.State.INAKE)
//                    instant {
//
//                    } else instant {
//                        println("balls")
//                    }
//        }


        driver[Button.Key.DPAD_DOWN] onActivate instant {
            dt.dt().IMU_OFFSET += dt.dt().rawExternalHeading
        }


        driver[Button.Key.LEFT_BUMPER] onActivate instant {
            smec.leftClawState = when (smec.leftClawState) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.CLOSED -> {
                    if (smec.state == ScoringMechanism.State.INTAKE)
                        ClawPositions.OPEN
                    else if (smec.state == ScoringMechanism.State.CLOSE_INTAKE )
                        ClawPositions.OPEN
                    else ClawPositions.DROP
                }
                ClawPositions.DROP -> ClawPositions.CLOSED
            }


        }

        driver[Button.Key.RIGHT_BUMPER] onActivate instant {
            smec.rightClawState = when (smec.rightClawState) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.DROP -> ClawPositions.CLOSED

                ClawPositions.CLOSED -> {
                    if (smec.state == ScoringMechanism.State.INTAKE )
                        ClawPositions.OPEN
                    else if (smec.state == ScoringMechanism.State.CLOSE_INTAKE )
                        ClawPositions.OPEN
                    else ClawPositions.DROP;
                }
            }
        }



        driverLeftTrigger onActivate
                instant {
                    if (smec.state == ScoringMechanism.State.CLOSE_INTAKE)
                        smec.state = ScoringMechanism.State.INTAKE

                    else if (smec.state == ScoringMechanism.State.INTAKE)
                        smec.state = ScoringMechanism.State.CLOSE_INTAKE
                    else
                        smec.pixelHeight -= 1.0
                }

        driverRightTrigger onActivate
                instant {
                        smec.pixelHeight += 1.0
                    }





        driver[Button.Key.LEFT_JOSTICK_PRESS] onActivate instant {
            if  (smec.state == ScoringMechanism.State.DEPOSIT)
                smec.state = ScoringMechanism.State.TRAVEL
            else if (smec.state == ScoringMechanism.State.INTAKE)
                smec.state = ScoringMechanism.State.TRAVEL
            else if (smec.state == ScoringMechanism.State.CLOSE_INTAKE)
                smec.state = ScoringMechanism.State.TRAVEL
            else
                smec.state = ScoringMechanism.State.CLOSE_INTAKE
                smec.rightClawState = ClawPositions.OPEN
                smec.leftClawState = ClawPositions.OPEN


        }



                parallel(
            instant {
                smec.switch(ScoringMechanism.State.CLOSE_INTAKE)
            },
            instant {
                smec.rightClawState = ClawPositions.OPEN
                smec.leftClawState = ClawPositions.OPEN
            }





        )



        driver[Button.Key.RIGHT_JOYSTICK_PRESS] onActivate instant {
            if  (smec.state == ScoringMechanism.State.DEPOSIT)
                smec.state = ScoringMechanism.State.TRAVEL
            else
                smec.state = ScoringMechanism.State.DEPOSIT
        }

        driver[Button.Key.SQUARE] onActivate
            series(
                instant {
                    smec.rightClawState = ClawPositions.CLOSED
                    smec.leftClawState = ClawPositions.CLOSED
                },
                delay(0.1),
                instant {
                    smec.switch(ScoringMechanism.State.TRAVEL)
                }
            )


        driver[Button.Key.CROSS] onActivate
            series(
                instant {
                    smec.rightClawState = ClawPositions.DROP
                    smec.leftClawState = ClawPositions.DROP
                },
                delay(0.25),
                instant {
                    smec.switch(ScoringMechanism.State.DROP)
                },
                instant {
                    smec.switch(ScoringMechanism.State.TRAVEL)
                }
            )

        driver[Button.Key.START] onActivate instant {
            if (smec.state == ScoringMechanism.State.CLIMB)
                + parallel(
                    instant {
                        smec.switch(ScoringMechanism.State.DROP)
                    },
                    series(
                        delay(0.75),
                        instant {
                            smec.switch(ScoringMechanism.State.TRAVEL)
                        }
                    )

                )
            else
                smec.switch(ScoringMechanism.State.CLIMB)

        }

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

            robot.telemetry.update()
        }
    }

    override fun run() {

    }
}