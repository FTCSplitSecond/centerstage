package org.firstinspires.ftc.teamcode.robot.opmodes

import LaunchDrone
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
import org.firstinspires.ftc.teamcode.claw.commands.DropBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.TriggerPositions
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.robot.util.adjustPowerForKStatic
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class MainTeleOp : AnchorOpMode() {
    lateinit var driver: FTCGamepad
    lateinit var robot: Robot

    override fun prerun() {
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, OpModeType.TELEOP)
        driver = FTCGamepad(gamepad1)
    }

    override fun run() {
        val smec = robot.scoringMechanism
        robot.init(this.world)

        schedule(
            instant {
                robot.droneLauncher.triggerPosition = TriggerPositions.HELD
            }
        )

        val shapeJoystickResponse : (Double) -> Double = {
            val shapedPower = it.absoluteValue.pow(DriveConstants.JOYSTICK_EXPONENT) * sign(it)
            shapedPower
        }
        val command = DriveMecanum(robot.driveBase,
            {
                val leftY = -driver[Button.Joystick.LEFT].y
                shapeJoystickResponse(leftY)
            },
            {
                val leftX = -driver[Button.Joystick.LEFT].x
                shapeJoystickResponse(leftX)
            },
            {
                val rightX = -driver[Button.Joystick.RIGHT].x * DriveConstants.OMEGA_WEIGHT
                if (smec.armState == ScoringMechanism.State.EXTENDED_INTAKE || smec.armState == ScoringMechanism.State.DEPOSIT) {
                    rightX * 0.5
                } else
                    rightX
                shapeJoystickResponse(rightX)
            })
        schedule(command)

        val triggerThreshold = 0.2
        val driverRightTrigger = EventTrigger { driver[Button.Trigger.RIGHT] > triggerThreshold }
        val driverLeftTrigger = EventTrigger { driver[Button.Trigger.LEFT] > triggerThreshold }

        driver[Button.Key.TRIANGLE] onActivate
                smec.setArmState(ScoringMechanism.State.STACK_INTAKE_CLOSE)

        val routine = RoutineModule { true }
        driver.apply(routine)

        driver[Button.Key.DPAD_DOWN] onActivate instant {
            robot.awayFromDriverStationHeading = robot.driveBase.dt.poseEstimate.heading
        }
        driver[Button.Key.LEFT_BUMPER] onActivate instant {
            robot.leftClaw.position = when (robot.leftClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.CLOSED -> {
                    when (smec.armState) {
                        ScoringMechanism.State.EXTENDED_INTAKE -> ClawPositions.OPEN
                        ScoringMechanism.State.CLOSE_INTAKE -> ClawPositions.OPEN
                        else -> ClawPositions.DROP
                    }
                }
                ClawPositions.DROP -> ClawPositions.CLOSED
            }
        }
        driver[Button.Key.RIGHT_BUMPER] onActivate instant {
            robot.rightClaw.position = when (robot.rightClaw.position) {
                ClawPositions.OPEN -> ClawPositions.CLOSED
                ClawPositions.DROP -> ClawPositions.CLOSED

                ClawPositions.CLOSED -> {
                    when (smec.armState) {
                        ScoringMechanism.State.EXTENDED_INTAKE -> ClawPositions.OPEN
                        ScoringMechanism.State.CLOSE_INTAKE -> ClawPositions.OPEN
                        else -> ClawPositions.DROP
                    };
                }
            }
        }
        driverLeftTrigger onActivate instant {
            +when (smec.armState) {
                ScoringMechanism.State.CLOSE_INTAKE -> parallel(
                    OpenBothClaw(robot.leftClaw, robot.rightClaw),
                    smec.setArmState(ScoringMechanism.State.EXTENDED_INTAKE))
                ScoringMechanism.State.EXTENDED_INTAKE -> smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
                else -> smec.setDepositPixelLevel(smec.depositPixelLevel - 1.0)
            }
        }
        driverRightTrigger onActivate instant {
            +when (smec.armState) {
                ScoringMechanism.State.CLOSE_INTAKE -> smec.setArmState(ScoringMechanism.State.EXTENDED_INTAKE)
                ScoringMechanism.State.EXTENDED_INTAKE -> smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
                else -> smec.setDepositPixelLevel(smec.depositPixelLevel + 1.0)
            }
        }
        driver[Button.Key.LEFT_JOSTICK_PRESS] onActivate
                instant {
                    +when (smec.armState) {
                        ScoringMechanism.State.DEPOSIT,
                        ScoringMechanism.State.EXTENDED_INTAKE,
                        ScoringMechanism.State.CLOSE_INTAKE -> smec.setArmState(ScoringMechanism.State.TRAVEL)
                        else -> smec.setArmState(ScoringMechanism.State.CLOSE_INTAKE)
                    }
                }
        driver[Button.Key.RIGHT_JOYSTICK_PRESS] onActivate instant {
            +when (smec.armState) {
                ScoringMechanism.State.TRAVEL -> smec.setArmState(ScoringMechanism.State.DEPOSIT)
                else -> smec.setArmState(ScoringMechanism.State.TRAVEL)
            }
        }
        driver[Button.Key.SQUARE] onActivate instant {
            +series(
                CloseBothClaw(robot.leftClaw, robot.rightClaw),
                smec.setArmState(ScoringMechanism.State.TRAVEL)
            )
        }
        driver[Button.Key.CROSS] onActivate instant {
                +series(
                    DropBothClaw(robot.leftClaw, robot.rightClaw),
                    delay(0.125),
                    smec.setArmState(ScoringMechanism.State.TRAVEL)
                )
            }
        driver[Button.Key.START] onActivate instant{
                +when (smec.armState) {
                    ScoringMechanism.State.CLIMB -> smec.setArmState(ScoringMechanism.State.TRAVEL)
                    else -> smec.setArmState(ScoringMechanism.State.CLIMB)
                }
        }
        driver[Button.Key.SHARE] onActivate LaunchDrone(robot.droneLauncher)

        +UpdateTelemetry(robot) {
            robot.telemetry.addData("X", robot.driveBase.poseEstimate().x)
            robot.telemetry.addData("Y", robot.driveBase.poseEstimate().y)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngle)
            robot.telemetry.addData("Wrist Angle", robot.wrist.angle)
            robot.telemetry.addData("Telescope Ext", robot.telescope.currentExtensionInches)
            robot.telemetry.addData("Left Claw", robot.leftClaw.position)
            robot.telemetry.addData("Light Claw", robot.rightClaw.position)
            robot.telemetry.addData("Arm State", smec.armState)
        }
    }

    override fun end() {
    }
}