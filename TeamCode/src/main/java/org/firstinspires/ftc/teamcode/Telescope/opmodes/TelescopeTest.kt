package org.firstinspires.ftc.teamcode.Telescope.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.Telescope.subsystems.TelescopePosition
import org.firstinspires.ftc.teamcode.Telescope.subsystems.TelescopeSubsytem
import org.firstinspires.ftc.teamcode.claw.commands.SetClawPosition
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType

@TeleOp
class TelescopeTest() : CommandOpMode() {
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        val motor = hardwareMap.get(DcMotorEx::class.java, "telescope")
        val telescope = TelescopeSubsytem(motor, telemetry, OpModeType.AUTONOMOUS)
        telescope.isTelemetryEnabled = true

        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)
        val gamePadX = driver.getGamepadButton(GamepadKeys.Button.X)
        val gamePadY = driver.getGamepadButton(GamepadKeys.Button.Y)
        gamePadA.whenPressed(SetTelescopePosition(telescope, TelescopePosition.TRAVEL))
        gamePadB.whenPressed(SetTelescopePosition(telescope, TelescopePosition.DEPOSIT))
        gamePadX.whenPressed(SetTelescopePosition(telescope, TelescopePosition.CLOSE_INTAKE))
        gamePadY.whenPressed(SetTelescopePosition(telescope, TelescopePosition.EXTENDED_INTAKE))

    }
}