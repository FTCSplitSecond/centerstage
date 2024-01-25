package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.turtles.anchor.entity.Entity
import dev.turtles.anchor.world.World
import dev.turtles.electriceel.wrapper.HardwareManager
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeSubsytem
import org.firstinspires.ftc.teamcode.claw.subsystems.LeftClawSubsystem
import org.firstinspires.ftc.teamcode.claw.subsystems.RightClawSubsystem
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem
import org.firstinspires.ftc.teamcode.mecanum.subsystems.MecanumDriveBase
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem
import kotlin.math.PI

enum class OpModeType {
    TELEOP, AUTONOMOUS
}
enum class Alliance {
    RED, BLUE
}

class Robot(val hardwareMap: HardwareMap, val hw: HardwareManager, t: Telemetry,
            val opModeType: OpModeType = OpModeType.TELEOP,
            var startPose : Pose2d = lastKnownAutoPose
) {

    companion object {
        var lastKnownAutoPose = Pose2d(0.0, 0.0, 0.0)
        var alliance = Alliance.RED
    }
    // this just uses the start pose assumption of the robot is facing the driver station

//    var awayFromDriverStationHeading = when (opModeType) {
//        OpModeType.TELEOP -> this.startPose.heading + PI
//        OpModeType.AUTONOMOUS -> when (alliance) {
//            Alliance.RED -> Math.PI / 2.0
//            Alliance.BLUE -> -Math.PI / 2.0
//        }
//    }

    var awayFromDriverStationHeading = when (alliance) {
            Alliance.RED -> Math.PI / 2.0
            Alliance.BLUE -> -Math.PI / 2.0
    }



    // add multiple telemetry here for dashboard here
    val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, t)

    val entity = Entity()

    // add subsystems here
    var driveBase = MecanumDriveBase(this)
    val leftClaw = LeftClawSubsystem(hw, telemetry)
    val rightClaw = RightClawSubsystem(hw, telemetry)
    val wrist = WristSubsystem(hw, telemetry)
    val telescope = TelescopeSubsytem(hw, this)
    val elbow = ElbowSubsystem(this, hw, telescope)
    val droneLauncher = DroneSubsystem(hw, telemetry)
    val scoringMechanism = ScoringMechanism(leftClaw, rightClaw, wrist, telescope, elbow);

    fun init(world: World) {
        entity.add(driveBase)
        entity.add(leftClaw)
        entity.add(rightClaw)
        entity.add(wrist)
        entity.add(telescope)
        entity.add(elbow)
        entity.add(droneLauncher)
        entity.add(scoringMechanism)

        world.add(entity)
    }
}