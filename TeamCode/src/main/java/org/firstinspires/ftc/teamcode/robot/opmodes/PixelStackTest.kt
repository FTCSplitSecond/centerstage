package org.firstinspires.ftc.teamcode.robot.opmodes

import PropZone
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.impl.FTCGamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.vision.processors.PixelStackDetector
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvWebcam

@Autonomous
class PixelStackTest : AnchorOpMode() {
    lateinit var robot : Robot
    lateinit var smec : ScoringMechanism
    lateinit var drive : CenterstageMecanumDrive
    lateinit var webcam : OpenCvWebcam
    var detector = PixelStackDetector(telemetry)
    var zone = PropZone.UNKNOWN
    override fun prerun() {
        val driver = FTCGamepad(gamepad1)
        robot = Robot(hardwareMap, this.hardwareManager, telemetry)
        smec = robot.scoringMechanism
        drive = robot.driveBase.dt
        robot.elbow.isEnabled = true
        robot.init(this.world)
        OpenBothClaw(robot.leftClaw, robot.rightClaw)
    }

    override fun run() {
        + series(
            instant {smec.state = ScoringMechanism.State.STACK_INTAKE}
        )
    }
}