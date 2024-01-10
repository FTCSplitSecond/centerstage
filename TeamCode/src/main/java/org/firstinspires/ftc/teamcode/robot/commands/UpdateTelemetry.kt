package org.firstinspires.ftc.teamcode.robot.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

class UpdateTelemetry(val robot : Robot, val printTelemetry : () -> Unit ) : SplitSecondComponent() {
    override fun loop() {
        printTelemetry()
        robot.telemetry.update()
    }

    override fun isComplete(): Boolean {
        return false
    }
}