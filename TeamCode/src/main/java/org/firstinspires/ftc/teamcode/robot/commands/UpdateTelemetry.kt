package org.firstinspires.ftc.teamcode.robot.commands

import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot

class UpdateTelemetry(val robot : Robot, val printTelemetry : () -> Unit ) : SplitSecondComponent() {
    override fun loop() {
        printTelemetry()
        robot.telemetry.addData("Loop time", this.deltaTime.toString() + " seconds")
        robot.telemetry.update()
    }

    override fun isComplete(): Boolean {
        return false
    }
}