package org.firstinspires.ftc.teamcode.robot.opmodes

import LaunchDrone
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DroneLaunch.Commands.SetDroneForInit
import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.claw.commands.ToggleLeftClaw
import org.firstinspires.ftc.teamcode.claw.commands.ToggleRightClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.robot.commands.DecreasePixelLevel
import org.firstinspires.ftc.teamcode.robot.commands.IncreasePixelLevel
import org.firstinspires.ftc.teamcode.robot.commands.MoveToCloseIntake
import org.firstinspires.ftc.teamcode.robot.commands.MoveToDeposit
import org.firstinspires.ftc.teamcode.robot.commands.MoveToExtendedIntake
import org.firstinspires.ftc.teamcode.robot.commands.MoveToPlace
import org.firstinspires.ftc.teamcode.robot.commands.MoveToTravel
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class MainTeleOp() : CommandOpMode() {
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val robot = Robot(hardwareMap, telemetry)
        robot.elbow.isEnabled = true

        schedule(SetDroneForInit(robot.droneLauncher))
        val command = DriveMecanum(robot.driveBase,
            { driver.leftY.absoluteValue.pow(3) * sign(driver.leftY) },
            { driver.leftX.absoluteValue.pow(3) * - sign(driver.leftX) },
            { - driver.rightX })
        schedule(command)

        val triggerThreshold = 0.2
        val driverRightTrigger = Trigger { driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > triggerThreshold }
        val driverLeftTrigger = Trigger { driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > triggerThreshold }
        val driverRightBumper = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        val driverLeftBumper = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        val driverTriangleButton = driver.getGamepadButton(GamepadKeys.Button.Y)
        val driverCircleButton = driver.getGamepadButton(GamepadKeys.Button.B)
        val driverSquareButton = driver.getGamepadButton(GamepadKeys.Button.X)
        val driverCrossButton = driver.getGamepadButton(GamepadKeys.Button.A)
        val driverDPADLeftButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val driverDPADUpButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val driverDPADDownButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        driverLeftTrigger.whenActive(MoveToExtendedIntake(robot))
            .whenInactive(SequentialCommandGroup(
                CloseBothClaw(robot.leftclaw, robot.rightClaw), WaitCommand(250), MoveToTravel(robot)))
        driverCircleButton.whenPressed(MoveToCloseIntake(robot))

        driverRightTrigger.toggleWhenActive(MoveToDeposit(robot))
        driverCrossButton.whenPressed(
            SequentialCommandGroup(
                OpenBothClaw(robot.leftclaw, robot.rightClaw),
                MoveToPlace(robot),
                ParallelCommandGroup(
                    CloseBothClaw(robot.leftclaw, robot.rightClaw),
                    MoveToTravel(robot)
                )
            )
        )





        driverLeftBumper.whenPressed(ToggleLeftClaw(LeftClawSubsystem(robot)))
        driverRightBumper.whenPressed(ToggleRightClaw(RightClawSubsystem(robot)))


        driverTriangleButton.whenPressed(MoveToTravel(robot))


        driverDPADUpButton.whenPressed(IncreasePixelLevel(robot))
        driverDPADDownButton.whenPressed(DecreasePixelLevel(robot))
        driverDPADLeftButton.whenPressed(LaunchDrone(robot.droneLauncher))



        schedule(UpdateTelemetry(robot) {
            robot.telemetry.addData("X", robot.driveBase.poseEstimate.x)
            robot.telemetry.addData("Y", robot.driveBase.poseEstimate.y)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngle)
            robot.telemetry.addData("Wrist Angle", robot.wrist.angle)
            robot.telemetry.addData("Telescope Ext", robot.telescope.currentExtensionInches)
            robot.telemetry.addData("pixel level", robot.elbow.pixelLevel)
            robot.telemetry.addData("left claw", robot.leftclaw.position)
            robot.telemetry.addData("right claw", robot.rightClaw.position)
            robot.telemetry.update()
        })
    }
}