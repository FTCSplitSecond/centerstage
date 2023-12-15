package org.firstinspires.ftc.teamcode.claw

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.claw.commands.SetClawPosition
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawSubsystem

@TeleOp
class ClawTest() : CommandOpMode() {
    override fun initialize() {
        val leftClawServo = hardwareMap.get(ServoImplEx::class.java, "leftClawServo")
        val rightClawServo = hardwareMap.get(ServoImplEx::class.java, "rightClawServo")
        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        val clawSubsystem = ClawSubsystem(leftClawServo, rightClawServo, telemetry)
        val driver = GamepadEx(gamepad1)

        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)
        gamePadA.whenPressed(SetClawPosition(clawSubsystem, ClawPositions.OPEN))
        gamePadB.whenPressed(SetClawPosition(clawSubsystem, ClawPositions.CLOSED))
    }

}