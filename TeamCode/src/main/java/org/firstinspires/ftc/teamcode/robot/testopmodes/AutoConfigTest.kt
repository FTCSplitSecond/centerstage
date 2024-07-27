package org.firstinspires.ftc.teamcode.robot.testopmodes

import PropDetector
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.idler
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.parallel
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.claw.commands.DropBothClaw
import org.firstinspires.ftc.teamcode.claw.subsystems.ClawPositions
import org.firstinspires.ftc.teamcode.mecanum.commands.TrajectoryFollower
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.robot.util.Alliance
import org.firstinspires.ftc.teamcode.robot.util.adjustForAlliance
import org.firstinspires.ftc.teamcode.vision.processors.PropZoneDetected
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.robot.util.AutoConfig
import org.firstinspires.ftc.teamcode.robot.util.ParkLocation
import org.firstinspires.ftc.teamcode.vision.AprilTagRelocalize
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


@Autonomous
class AutoConfigTest : AnchorOpMode() {
    lateinit var robot: Robot

    lateinit var parkLocation: ParkLocation
    var delayA: Double = 0.0

    var isRunning = false

    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        robot.init(this.world)

        parkLocation = AutoConfig.RED_CLOSE2P0_PARK
        delayA = AutoConfig.RED_CLOSE2P0_DELAYS[0]

        + idler { _, _ ->
            telemetry.addLine(when(parkLocation) {
                ParkLocation.INSIDE -> "Park Inside"
                ParkLocation.CENTER -> "Park Center"
                ParkLocation.OUTSIDE -> "Park Outside"
            })
            telemetry.addLine("Start Pose Delay: $delayA")
            isRunning
        }

        driver[Button.Key.SQUARE] onActivate instant {
            parkLocation = when(parkLocation) {
                ParkLocation.INSIDE -> ParkLocation.CENTER
                ParkLocation.CENTER -> ParkLocation.OUTSIDE
                ParkLocation.OUTSIDE -> ParkLocation.INSIDE
            }
        }
        driver[Button.Key.CIRCLE] onActivate instant {
            parkLocation = when(parkLocation) {
                ParkLocation.INSIDE -> ParkLocation.OUTSIDE
                ParkLocation.CENTER -> ParkLocation.INSIDE
                ParkLocation.OUTSIDE -> ParkLocation.CENTER
            }
        }
        driver[Button.Key.TRIANGLE] onActivate instant {
            delayA = (delayA + 1.0).coerceIn(0.0, 30.0)
        }
        driver[Button.Key.CROSS] onActivate instant {
            delayA = (delayA - 1.0).coerceIn(0.0, 30.0)
        }
    }
    override fun run() {
        isRunning = true;
    }
}