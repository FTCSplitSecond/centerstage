package org.firstinspires.ftc.teamcode.robot.testopmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.electriceel.opmode.AnchorOpMode
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition

@Autonomous
class KgTester : AnchorOpMode() {
    val robot = Robot(hardwareMap, this.hardwareManager, telemetry)
    override fun prerun() {
        SetTelescopePosition(robot.telescope, TelescopePosition.Adjust(TelescopeConfig.TELESCOPE_MAX))
    }

    override fun run() {
        SetElbowPosition(robot.elbow, ElbowPosition.Adjust(0.0))
    }
}