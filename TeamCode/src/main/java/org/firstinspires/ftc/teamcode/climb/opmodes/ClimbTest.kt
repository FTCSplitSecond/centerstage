package org.firstinspires.ftc.teamcode.claw.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.RaiseClimbArms
import org.firstinspires.ftc.teamcode.claw.commands.SuspendRobot
import org.firstinspires.ftc.teamcode.claw.commands.ToggleLeftClaw
import org.firstinspires.ftc.teamcode.claw.commands.ToggleRightClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbArmSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.ClimbPulleySubsystem

@TeleOp
class ClimbTest() : CommandOpMode() {
    override fun initialize() {
//        val leftHookServo = hardwareMap.get(ServoImplEx::class.java, "leftHookServo")
//        val rightHookServo = hardwareMap.get(ServoImplEx::class.java, "rightHookServo")
//        val leftPulley = hardwareMap.get(CRServoImplEx::class.java, "leftClimbServo")
//        val rightPulley = hardwareMap.get(CRServoImplEx::class.java, "rightClimbServo")
//        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
//        val bothArms = ClimbArmSubsystem(leftHookServo, rightHookServo, telemetry)
//        val bothPulley = ClimbPulleySubsystem(leftPulley, rightPulley, telemetry)
//        val driver = GamepadEx(gamepad1)
//
//        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
//        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)
//        val gamePadY = driver.getGamepadButton(GamepadKeys.Button.Y)
//        val gamePadX = driver.getGamepadButton(GamepadKeys.Button.X)
//
//
//        gamePadA.whenPressed(RaiseClimbArms(bothArms))
//        gamePadX.whenPressed(SuspendRobot(bothPulley))
//

    }

}