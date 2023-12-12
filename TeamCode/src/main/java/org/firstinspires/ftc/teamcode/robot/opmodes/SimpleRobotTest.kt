package org.firstinspires.ftc.teamcode.robot.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class SimpleRobotTest() : CommandOpMode() {
    private val r = Robot(hardwareMap, telemetry)
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val robot = Robot(hardwareMap, telemetry, OpModeType.TELEOP)

        val command = DriveMecanum(robot.driveBase,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
            { driver.rightX })
        schedule(command)
    }
}