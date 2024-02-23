package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.turtles.anchor.entity.Entity
import dev.turtles.anchor.world.World
import dev.turtles.electriceel.wrapper.HardwareManager
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem
import org.firstinspires.ftc.teamcode.mecanum.subsystems.MecanumDriveBase
import org.firstinspires.ftc.teamcode.robot.util.Alliance
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.vision.subsystems.VisionSubsystem
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem


class Robot(val hardwareMap: HardwareMap, val hw: HardwareManager, t: Telemetry,
            val opModeType: OpModeType = OpModeType.TELEOP,
            var startPose : Pose2d = lastKnownAutoPose
) {

    companion object {
        var lastKnownAutoPose = Pose2d(0.0, 0.0, 0.0)
        var alliance = Alliance.RED
    }

    var awayFromDriverStationHeading = when (alliance) {
            Alliance.RED -> -Math.PI / 2.0
            Alliance.BLUE -> Math.PI / 2.0
    }

    // add multiple telemetry here for dashboard here
    val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, t)

    val entity = Entity()

    // add subsystems here
    var driveBase = MecanumDriveBase(this)
    val leftClaw = LeftClawSubsystem(this, hw, telemetry)
    val rightClaw = RightClawSubsystem(this, hw, telemetry)
    val wrist = WristSubsystem(hw, telemetry)
    val telescope = TelescopeSubsytem(hw, this)
    val elbow = ElbowSubsystem(this, hw, telescope)
    val droneLauncher = DroneSubsystem(hw, telemetry)
    val scoringMechanism = ScoringMechanism(leftClaw, rightClaw, wrist, telescope, elbow, telemetry);
    val vision = VisionSubsystem(this)

    fun init(world: World) {
        entity.add(driveBase)
        entity.add(leftClaw)
        entity.add(rightClaw)
        entity.add(wrist)
        entity.add(telescope)
        entity.add(elbow)
        entity.add(droneLauncher)
        world.add(entity)

    }
}