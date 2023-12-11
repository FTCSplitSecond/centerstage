package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

enum class OpModeType {
    TELEOP, AUTONOMOUS
}
class Robot(val hardwareMap: HardwareMap, t: Telemetry, val opModeType: OpModeType = OpModeType.TELEOP) {

    val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, t)


}