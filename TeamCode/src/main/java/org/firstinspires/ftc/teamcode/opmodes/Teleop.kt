package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.idler
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.DSLOpMode
import dev.turtles.electriceel.util.Pose
import dev.turtles.electriceel.util.Vector2
import dev.turtles.electriceel.util.radians
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.EventTrigger
import dev.turtles.lilypad.impl.FTCGamepad
import dev.turtles.lilypad.module.RoutineModule
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.OffseasonBot
import org.firstinspires.ftc.teamcode.common.component.DriveMecanum
import org.firstinspires.ftc.teamcode.common.types.OpModeType
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "Primary TeleOp", group = "!")
class Teleop: DSLOpMode(false, {
    val robot = OffseasonBot(
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

    + idler { _, _ ->
        false
    }

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
