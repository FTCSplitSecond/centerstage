package org.firstinspires.ftc.teamcode.mecanum.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.mecanum.commands.OutputDriveTelemetry
import org.firstinspires.ftc.teamcode.mecanum.subsystems.MecanumDriveBase
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
public class MecanumDriveTest : CommandOpMode() {

    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val multiTelemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        val mecanumDrive = MecanumDriveBase(hardwareMap, multiTelemetry)
        val command = DriveMecanum(mecanumDrive,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * - sign(driver.leftX) },
            { - driver.rightX })
        schedule(command)
        schedule(OutputDriveTelemetry(multiTelemetry, mecanumDrive))
    }



}