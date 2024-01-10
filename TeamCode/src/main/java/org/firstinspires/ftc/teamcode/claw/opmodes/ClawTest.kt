package org.firstinspires.ftc.teamcode.claw.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.ToggleLeftClaw
import org.firstinspires.ftc.teamcode.claw.commands.ToggleRightClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem

@TeleOp
class ClawTest() : CommandOpMode() {
    override fun initialize() {
//        val leftClawServo = hardwareMap.get(ServoImplEx::class.java, "leftClawServo")
//        val rightClawServo = hardwareMap.get(ServoImplEx::class.java, "rightClawServo")
//        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
//        val leftClaw = LeftClawSubsystem(leftClawServo, telemetry)
//        val rightClaw = RightClawSubsystem(rightClawServo, telemetry)
//        val driver = GamepadEx(gamepad1)
//
//        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
//        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)
//        val gamePadY = driver.getGamepadButton(GamepadKeys.Button.Y)
//        val gamePadX = driver.getGamepadButton(GamepadKeys.Button.X)
//        gamePadA.whenPressed(OpenBothClaw(leftClaw, rightClaw))
//        gamePadY.whenPressed(CloseBothClaw(leftClaw, rightClaw))
//        gamePadX.whenPressed(ToggleLeftClaw(leftClaw))
//        gamePadB.whenPressed((ToggleRightClaw(rightClaw)))
    }

}