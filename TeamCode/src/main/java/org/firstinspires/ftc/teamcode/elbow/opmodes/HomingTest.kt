package org.firstinspires.ftc.teamcode.elbow.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class HomingTest() : CommandOpMode() {
    override fun initialize() {
//        val driver = GamepadEx(gamepad1)
//        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
//        val motor = hardwareMap.get(DcMotorEx::class.java, "elbow")
//        val elbow = ElbowSubsystem(motor, telemetry, OpModeType.AUTONOMOUS)
//        elbow.isTelemetryEnabled = true
//        elbow.isEnabled = true
//        ElbowConfig.ELBOW_MIN = -10.0
//
//        val gamePadA = driver.getGamepadButton(GamepadKeys.Button.A)
//        val gamePadB = driver.getGamepadButton(GamepadKeys.Button.B)
//        val gamePadX = driver.getGamepadButton(GamepadKeys.Button.X)
//        //val gamePadY = driver.getGamepadButton(GamepadKeys.Button.Y)
////        gamePadA.whenPressed(InstantCommand({ elbow.targetAngle -= 1.0 }))
////        gamePadB.whenPressed(InstantCommand({ elbow.targetAngle += 1.0 }))
//        gamePadX.whenPressed(SetElbowPosition(elbow, ElbowPosition.HOME))
//
    }
}