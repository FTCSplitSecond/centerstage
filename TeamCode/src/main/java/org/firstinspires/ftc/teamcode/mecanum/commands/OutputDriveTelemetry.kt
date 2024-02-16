package org.firstinspires.ftc.teamcode.mecanum.commands

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.mecanum.subsystems.MecanumDriveBase

class OutputDriveTelemetry(val telemetry: Telemetry, val drive : MecanumDriveBase) : SplitSecondComponent() {
    override fun loop() {
        telemetry.addData("pose.heading", drive.poseEstimate().heading)
        telemetry.addData("imuheading", drive.rawExternalHeading())
        telemetry.addData("pose.x", drive.poseEstimate().x)
        telemetry.addData("pose.y", drive.poseEstimate().y)
        telemetry.update()
    }
}