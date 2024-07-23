package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.idler
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.DSLOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.EventTrigger
import dev.turtles.lilypad.impl.FTCGamepad
import dev.turtles.lilypad.module.RoutineModule
import org.firstinspires.ftc.teamcode.SplitSecondBot
import org.firstinspires.ftc.teamcode.common.component.DriveMecanum
import org.firstinspires.ftc.teamcode.common.types.ClawSide
import org.firstinspires.ftc.teamcode.common.types.OpModeType
import org.firstinspires.ftc.teamcode.component.claw.CloseBothClaw
import org.firstinspires.ftc.teamcode.component.claw.DropBothClaw
import org.firstinspires.ftc.teamcode.component.claw.OpenBothClaw
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "Primary TeleOp", group = "!")
class Teleop: DSLOpMode(false, {
    val robot = SplitSecondBot(
        hardwareMap,
        this.hardwareManager,
        OpModeType.TELE_OP,
        Pose2d(0.0, 0.0, 0.0),
        telemetry
    )

    val driver = FTCGamepad(gamepad1)
    val dt = robot.drivetrain

    robot.init(world)

    + robot.robot

    val deposit = robot.deposit
    val drone = robot.drone

    drone.init()

    + idler { deltaTime, _ ->
        // pray that this is above 25 :3
        telemetry.addData("fps", 1.0 / deltaTime)
        telemetry.update()

        false
    }

    val triggerThresh = 0.2
    val driverRightTrigger = EventTrigger { driver[Button.Trigger.RIGHT] > triggerThresh }
    val driverLeftTrigger = EventTrigger { driver[Button.Trigger.LEFT] > triggerThresh }

    driver[Button.Key.TRIANGLE] onActivate deposit.setArmState(DepositSubsystem.State.STACK_INTAKE_CLOSED)

    val shapeJoystickResponse: (Double) -> Double = {
        val shapedPower = it.absoluteValue.pow(DriveConstants.JOYSTICK_EXPONENT) * sign(it)
        shapedPower
    }

    val command = DriveMecanum(
        robot.drivetrain,
        {
            val leftY = -driver[Button.Joystick.LEFT].y
            shapeJoystickResponse(leftY)
        },
        {
            val leftX = driver[Button.Joystick.LEFT].x
            shapeJoystickResponse(leftX)
        },
        {
            val rightX = driver[Button.Joystick.RIGHT].y * DriveConstants.OMEGA_WEIGHT
            if (deposit.armState == DepositSubsystem.State.EXTENDED_INTAKE || deposit.armState == DepositSubsystem.State.DEPOSIT) {
                rightX * 0.5
            }

            shapeJoystickResponse(rightX)
        }
    )

    schedule(command)

    val routine = RoutineModule { true }
    driver.apply(routine)

    driver[Button.Key.TRIANGLE] onActivate instant { deposit.setArmState(DepositSubsystem.State.STACK_INTAKE_CLOSED) }

    driver[Button.Key.LEFT_BUMPER] onActivate instant {
        when (robot.claw.leftClaw) {
            ClawSubsystem.ClawState.OPEN -> robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.CLOSED)
            ClawSubsystem.ClawState.CLOSED -> {
                when (deposit.armState) {
                    DepositSubsystem.State.EXTENDED_INTAKE -> robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.OPEN)
                    DepositSubsystem.State.CLOSED_INTAKE -> robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.OPEN)
                    else -> robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.CLOSED)
                }
            }
            else -> robot.claw.updateState(ClawSide.LEFT, ClawSubsystem.ClawState.CLOSED)
        }
    }

    driver[Button.Key.RIGHT_BUMPER] onActivate instant {
        when (robot.claw.rightClaw) {
            ClawSubsystem.ClawState.OPEN -> robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.CLOSED)
            ClawSubsystem.ClawState.CLOSED -> {
                when (deposit.armState) {
                    DepositSubsystem.State.EXTENDED_INTAKE -> robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.OPEN)
                    DepositSubsystem.State.CLOSED_INTAKE -> robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.OPEN)
                    else -> robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.CLOSED)
                }
            }
            else -> robot.claw.updateState(ClawSide.RIGHT, ClawSubsystem.ClawState.CLOSED)
        }
    }

    driverLeftTrigger onActivate instant {
        + when (deposit.armState) {
            DepositSubsystem.State.CLOSED_INTAKE -> parallel(
                OpenBothClaw(robot.claw),
                deposit.setArmState(DepositSubsystem.State.EXTENDED_INTAKE)
            )
            DepositSubsystem.State.EXTENDED_INTAKE -> deposit.setArmState(DepositSubsystem.State.CLOSED_INTAKE)
            else -> deposit.setPixelLevel(deposit.depositPixelLevel - 1.0)
        }
    }

    driverRightTrigger onActivate instant {
        + when (deposit.armState) {
            DepositSubsystem.State.CLOSED_INTAKE -> deposit.setArmState(DepositSubsystem.State.EXTENDED_INTAKE)
            DepositSubsystem.State.EXTENDED_INTAKE -> deposit.setArmState(DepositSubsystem.State.CLOSED_INTAKE)
            else -> deposit.setPixelLevel(deposit.depositPixelLevel + 1.0)
        }
    }

    driver[Button.Key.LEFT_JOSTICK_PRESS] onActivate instant {
        + when (deposit.armState) {
            DepositSubsystem.State.DEPOSIT,
            DepositSubsystem.State.EXTENDED_INTAKE,
            DepositSubsystem.State.CLOSED_INTAKE -> deposit.setArmState(DepositSubsystem.State.TRAVEL)
            else -> deposit.setArmState(DepositSubsystem.State.CLOSED_INTAKE)
        }
    }

    driver[Button.Key.RIGHT_JOYSTICK_PRESS] onActivate instant {
        + when (deposit.armState) {
            DepositSubsystem.State.TRAVEL -> deposit.setArmState(DepositSubsystem.State.DEPOSIT)
            else -> deposit.setArmState(DepositSubsystem.State.TRAVEL)
        }
    }

    driver[Button.Key.SQUARE] onActivate instant {
        + series(
            CloseBothClaw(robot.claw),
            deposit.setArmState(DepositSubsystem.State.TRAVEL)
        )
    }

    driver[Button.Key.CROSS] onActivate instant {
        + series(
            DropBothClaw(robot.claw),
            delay(0.125),
            deposit.setArmState(DepositSubsystem.State.TRAVEL)
        )
    }

    driver[Button.Key.START] onActivate instant {
        + when (deposit.armState) {
            DepositSubsystem.State.CLIMB -> deposit.setArmState(DepositSubsystem.State.TRAVEL)
            else -> deposit.setArmState(DepositSubsystem.State.CLIMB)
        }
    }

    driver[Button.Key.DPAD_DOWN] onActivate instant {
        robot.driverStationOffset = robot.drivetrain.poseEstimate().heading
    }

    driver[Button.Key.SHARE] onActivate instant {
        drone.pitchPosition = DroneSubsystem.PitchPositions.LAUNCHED
    }

    driver[Button.Key.SHARE] onDeactivate series(
        instant {
            drone.launched = true
        },
        delay(0.25),
        instant { drone.pitchPosition = DroneSubsystem.PitchPositions.STOWED }
    )
})
