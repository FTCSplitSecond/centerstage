package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.idler
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
import org.firstinspires.ftc.teamcode.common.types.OpModeType

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
    val imu = robot.imu
    val dt = robot.drivetrain

    robot.init(world)

    + robot.robot

    val deposit = robot.deposit
    val drone = robot.drone

    drone.init()

    var offset = 0.0
    fun raw() = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.RADIANS).firstAngle

    fun resetIMU() {
        offset = (-raw()).toDouble()
    }

    fun heading() =
        (imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + offset).radians.normalize()

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

    val routine = RoutineModule { true }
    driver.apply(routine)


})
