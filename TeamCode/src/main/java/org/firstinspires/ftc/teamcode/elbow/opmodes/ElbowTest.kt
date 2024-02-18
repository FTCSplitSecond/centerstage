package org.firstinspires.ftc.teamcode.elbow.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.instant
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition

@TeleOp
class ElbowTest() : AnchorOpMode() {
    lateinit var driver: FTCGamepad
    lateinit var robot: Robot
    override fun prerun() {
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, OpModeType.TELEOP)
        driver = FTCGamepad(gamepad1)
        robot.init(this.world)

        +robot.scoringMechanism.setArmState(ScoringMechanism.State.TRAVEL)
    }

    override fun run() {

        driver[Button.Key.DPAD_DOWN] onActivate instant {
            +SetElbowPosition(robot.elbow, ElbowPosition.Adjust(robot.elbow.targetAngle - ElbowConfig.ELBOW_TEST_INCREMENT))
        }
        driver[Button.Key.DPAD_UP] onActivate instant {
            +SetElbowPosition(robot.elbow, ElbowPosition.Adjust(robot.elbow.targetAngle + ElbowConfig.ELBOW_TEST_INCREMENT))
        }

        +UpdateTelemetry(robot) {
            robot.telemetry.addData("Elbow Kg: ", ElbowConfig.KG)
            robot.telemetry.addData("Elbow Ks: ", ElbowConfig.KS)
            robot.telemetry.addData("Elbow Target Angle", robot.elbow.targetAngle)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngle)
            robot.telemetry.addData("Wrist Angle", robot.wrist.angle)
            robot.telemetry.addData("Telescope Ext", robot.telescope.currentExtensionInches)

        }
    }
}