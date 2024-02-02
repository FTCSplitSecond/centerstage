package org.firstinspires.ftc.teamcode.robot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.vision.AprilTagRelocalizer

@Autonomous
class AprilTagRelocalizationTest : AnchorOpMode() {
    lateinit var driver : FTCGamepad
    override fun prerun() {
        val webcamName = hardwareMap.get(WebcamName::class.java, "webcam1")
        val robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        robot.init(this.world)
        driver = FTCGamepad(gamepad1)

        AprilTagRelocalizer.initAprilTag(webcamName)
        driver[Button.Key.DPAD_DOWN] onActivate series(AprilTagRelocalizer(robot), instant {telemetry.update()})
//        driver[Button.Key.DPAD_DOWN] onActivate instant {
//            telemetry.addLine(num.toString())
//            num += 1
//            telemetry.update()
//        }
    }

    override fun run() {
    }
}