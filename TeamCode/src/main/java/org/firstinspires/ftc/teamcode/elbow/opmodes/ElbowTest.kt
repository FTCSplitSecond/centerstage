package org.firstinspires.ftc.teamcode.elbow.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem

@TeleOp
class ElbowTest() : CommandOpMode() {
    override fun initialize() {

        val robot = Robot(hardwareMap, telemetry)
        val driver = GamepadEx(gamepad1)
        robot.elbow.isTelemetryEnabled = true
        robot.elbow.isEnabled = false

        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)
        val gamePadX = driver.getGamepadButton(GamepadKeys.Button.X)
        val gamePadY = driver.getGamepadButton(GamepadKeys.Button.Y)
        gamePadA.whenPressed(SetElbowPosition(robot.elbow, ElbowPosition.TRAVEL))
        gamePadB.whenPressed(SetElbowPosition(robot.elbow, ElbowPosition.DEPOSIT))
        gamePadX.whenPressed(SetElbowPosition(robot.elbow, ElbowPosition.CLOSE_INTAKE))
        gamePadY.whenPressed(SetElbowPosition(robot.elbow, ElbowPosition.EXTENDED_INTAKE))

    }
}