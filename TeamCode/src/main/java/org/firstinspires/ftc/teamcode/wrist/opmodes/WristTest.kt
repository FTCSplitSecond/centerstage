package org.firstinspires.ftc.teamcode.wrist.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPositions
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem

@TeleOp
class WristTest(): CommandOpMode() {
    override fun initialize() {
        val leftWristServo = hardwareMap.get(ServoImplEx::class.java, "leftWristServo")
        val rightWristServo = hardwareMap.get(ServoImplEx::class.java, "rightWristServo")
        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        val wristSubsystem = WristSubsystem(leftWristServo, rightWristServo, telemetry)
        val driver = GamepadEx(gamepad1)

        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)

        gamePadA.whenPressed(SetWristPosition(wristSubsystem, WristPositions.UP))
        gamePadB.whenPressed(SetWristPosition(wristSubsystem, WristPositions.REST))
    }
}