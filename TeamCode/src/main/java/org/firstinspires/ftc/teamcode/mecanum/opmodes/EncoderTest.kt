package org.firstinspires.ftc.teamcode.mecanum.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder

@TeleOp
class EncoderTest() : LinearOpMode() {
    override fun runOpMode() {
        val leftEncoder = Encoder(hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "fL"))
        val rightEncoder = Encoder(hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "bR"))
        rightEncoder.direction = Encoder.Direction.REVERSE

        val frontEncoder = Encoder(hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "bL"))

        val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        waitForStart()

        val leftStart = leftEncoder.currentPosition
        val rightStart = rightEncoder.currentPosition
        val frontStart = frontEncoder.currentPosition

        while (opModeIsActive()) {
            // Get the current position of the motor

            // Show the position of the motor on telemetry
            telemetry.addData("leftEncoder", leftEncoder.currentPosition - leftStart)
            telemetry.addData("rightEncoder", rightEncoder.currentPosition - rightStart)
            telemetry.addData("frontEncoder", frontEncoder.currentPosition - frontStart)
            telemetry.update()
        }
    }
}