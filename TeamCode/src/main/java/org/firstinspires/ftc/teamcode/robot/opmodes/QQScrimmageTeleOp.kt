package org.firstinspires.ftc.teamcode.robot.opmodes

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.claw.commands.SetClawPosition
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.robot.commands.MoveToCloseIntake
import org.firstinspires.ftc.teamcode.robot.commands.MoveToDeposit
import org.firstinspires.ftc.teamcode.robot.commands.MoveToDepositSafe
import org.firstinspires.ftc.teamcode.robot.commands.MoveToExtendedIntake
import org.firstinspires.ftc.teamcode.robot.commands.MoveToTravel
import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class QQScrimmageTeleOp() : CommandOpMode() {
    override fun initialize() {
        val driver = GamepadEx(gamepad1)
        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)

        val command = DriveMecanum(robot.driveBase,
            { driver.leftY.pow(2) * sign(driver.leftY) },
            { driver.leftX.pow(2) * sign(driver.leftX) },
            { driver.rightX })
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

        driverLeftTrigger.whenActive(MoveToExtendedIntake(robot)).whenInactive(MoveToTravel(robot))
        driverLeftBumper.whenActive(MoveToCloseIntake(robot)).whenInactive(MoveToTravel(robot))
        driverRightTrigger.whenActive(MoveToDeposit(robot)).whenInactive(SequentialCommandGroup(
            SetClawPosition(robot.claw, ClawPositions.OPEN), WaitCommand(250), MoveToTravel(robot)))
        driverRightBumper.whenActive(MoveToDepositSafe(robot)).whenInactive(SequentialCommandGroup(
            SetClawPosition(robot.claw, ClawPositions.OPEN), WaitCommand(250), MoveToTravel(robot)))
        driverTriangleButton.whenPressed(MoveToTravel(robot))


    }
}