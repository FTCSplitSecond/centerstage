package org.firstinspires.ftc.teamcode.wrist.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.wrist.commands.SetWristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristPosition
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem

@TeleOp
class WristTest : AnchorOpMode() {
    lateinit var robot : Robot
    override fun prerun() {
        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        robot.init(this.world)
        robot.elbow.isTelemetryEnabled = true
        robot.wrist.isTelemetryEnabled = true
    }

    override fun run() {
        + series(
            SetWristPosition(robot.wrist, WristPosition.CloseIntake),
            delay(1.0),
            SetWristPosition(robot.wrist, WristPosition.Travel),
        )
        + UpdateTelemetry(robot) {}
    }
}