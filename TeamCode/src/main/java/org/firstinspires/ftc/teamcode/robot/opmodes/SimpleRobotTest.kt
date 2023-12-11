package org.firstinspires.ftc.teamcode.robot.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.RobotCS

@TeleOp
class SimpleRobotTest() : CommandOpMode() {
    private val r = RobotCS(hardwareMap, telemetry)
    override fun initialize() {
        r.telemetry.addLine("Initializing...")
        r.telemetry.update()
        val command = InstantCommand({
            r.telemetry.addLine("running command")
            r.telemetry.update()
        })
        schedule(command)
    }
}