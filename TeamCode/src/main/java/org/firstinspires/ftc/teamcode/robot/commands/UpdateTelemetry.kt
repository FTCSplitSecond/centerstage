package org.firstinspires.ftc.teamcode.robot.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

class UpdateTelemetry(val robot : Robot, val printTelemetry : () -> Unit ) : CommandBase() {
        override fun execute() {
            printTelemetry()
            robot.telemetry.update()
        }
        override fun isFinished(): Boolean {
            return false
        }
    }