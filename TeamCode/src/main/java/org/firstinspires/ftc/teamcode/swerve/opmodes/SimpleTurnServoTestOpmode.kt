package org.firstinspires.ftc.teamcode.swerve.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.swerve.commands.TestPodServos


@TeleOp(name = "Leapfrog Servo Test", group = "none")

class SimpleTurnServoTestOpmode() : CommandOpMode(){
    override fun initialize() {
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val fL = CRServo(hardwareMap, "frontLeftServo")
        val fR = CRServo(hardwareMap, "frontRightServo")
        val bL = CRServo(hardwareMap, "backLeftServo")
        val bR = CRServo(hardwareMap, "backRightServo")
        val allServos = MotorGroup(fL,fR, bL, bR)

        val fLA = hardwareMap.get(AnalogInput::class.java, "frontLeftServoAngle")
        val fRA = hardwareMap.get(AnalogInput::class.java, "frontRightServoAngle")
        val bLA = hardwareMap.get(AnalogInput::class.java,"backLeftServoAngle")
        val bRA = hardwareMap.get(AnalogInput::class.java,"backRightServoAngle")

        val driverOp = GamepadEx(gamepad1);
        val command = TestPodServos(allServos, fLA, fRA, bLA, bRA, telemetry) { Rotation2d(driverOp.leftY, driverOp.leftX) }

        schedule(command)
    }


}