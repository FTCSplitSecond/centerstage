package org.firstinspires.ftc.teamcode.robot.opmodes

import LaunchDrone
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.stock.forever
import dev.turtles.anchor.component.stock.instant
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.teamcode.drone_launcher.Commands.SetDroneForInit
import org.firstinspires.ftc.teamcode.mecanum.commands.DriveMecanum
import org.firstinspires.ftc.teamcode.robot.commands.MoveToTravel
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

@TeleOp
class MainTeleOp : AnchorOpMode() {
    override fun prerun() {
        val driver = FTCGamepad(gamepad1)

        val robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        val smec = robot.scoringMechanism

        robot.init(this.world)

        robot.elbow.isEnabled = true

        schedule(SetDroneForInit(robot.droneLauncher))

        val command = DriveMecanum(robot.driveBase,
            {
                val leftY = driver[Button.Joystick.LEFT].y
                leftY.absoluteValue.pow(3) * sign(leftY)
            },
            {
                val leftX = driver[Button.Joystick.LEFT].x
                leftX.absoluteValue.pow(3) * - sign(leftX)
            },
            { - driver[Button.Joystick.RIGHT].x })

        schedule(command)

        val triggerThreshold = 0.2

//        val driverRightTrigger = EventTrigger { driver[Button.Trigger.RIGHT] > triggerThreshold }
//        val driverLeftTrigger = EventTrigger { driver[Button.Trigger.LEFT] > triggerThreshold }


//        driverLeftTrigger onActivate (MoveToExtendedIntake(robot)) onDeactivate series(
//                    CloseBothClaw(robot.leftclaw, robot.rightClaw),
//                    delay(0.25),
//                    MoveToTravel(robot))
//        driver[Button.Key.CIRCLE] onActivate MoveToCloseIntake(robot)
//
//        driverRightTrigger onActivate MoveToDeposit(robot)
//
//        driver[Button.Key.CROSS] onActivate (
//            series(
//                OpenBothClaw(robot.leftclaw, robot.rightClaw),
//                MoveToPlace(robot),
//                parallel(
//                    CloseBothClaw(robot.leftclaw, robot.rightClaw),
//                    MoveToTravel(robot)
//                )
//            )
//        )
//
//        driver[Button.Key.LEFT_BUMPER] onActivate ToggleLeftClaw(robot.leftclaw)
//        driver[Button.Key.RIGHT_BUMPER] onActivate ToggleRightClaw(robot.rightClaw)
//
//        driver[Button.Key.TRIANGLE] onActivate MoveToTravel(robot)
//        driver[Button.Key.DPAD_UP] onActivate IncreasePixelLevel(robot)
//        driver[Button.Key.DPAD_DOWN] onActivate DecreasePixelLevel(robot)
//        driver[Button.Key.DPAD_LEFT] onActivate LaunchDrone(robot.droneLauncher)


        driver[Button.Key.SQUARE] onActivate instant {
            smec.switch(ScoringMechanism.State.INTAKE)
        }

        driver[Button.Key.DPAD_UP] onActivate instant {
            smec.switch(ScoringMechanism.State.DEPOSIT)
        }

        driver[Button.Key.CROSS] onActivate instant {
            smec.switch(ScoringMechanism.State.TRAVEL)
        }

        + forever { _, _ ->
            smec.elbowTesting = driver[Button.Trigger.RIGHT] - driver[Button.Trigger.LEFT]
        }

        + UpdateTelemetry(robot) {
            robot.telemetry.addData("X", robot.driveBase.poseEstimate().x)
            robot.telemetry.addData("Y", robot.driveBase.poseEstimate().y)
            robot.telemetry.addData("Elbow Angle", robot.elbow.currentAngle)
            robot.telemetry.addData("Wrist Angle", robot.wrist.angle)
            robot.telemetry.addData("Telescope Ext", robot.telescope.currentExtensionInches)
            robot.telemetry.addData("left claw", robot.leftClaw.position)
            robot.telemetry.addData("right claw", robot.rightClaw.position)

            robot.telemetry.update()
        }
    }

    override fun run() {

    }
}