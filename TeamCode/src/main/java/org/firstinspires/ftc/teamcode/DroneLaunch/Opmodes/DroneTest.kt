package org.firstinspires.ftc.teamcode.claw.opmodes

import LaunchDrone
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.DroneLaunch.Subsystems.DroneSubsystem

@TeleOp
class DroneTest() : CommandOpMode() {
    override fun initialize() {
        val droneServo = hardwareMap.get(ServoImplEx::class.java, "droneServo")
        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        val drone = DroneSubsystem(droneServo, telemetry)
        val driver = GamepadEx(gamepad1)

        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)
        val gamePadY = driver.getGamepadButton(GamepadKeys.Button.Y)
        val gamePadX = driver.getGamepadButton(GamepadKeys.Button.X)
        gamePadA.whenPressed(LaunchDrone(drone))

    }

}