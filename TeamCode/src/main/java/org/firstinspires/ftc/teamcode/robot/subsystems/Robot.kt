package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
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
import org.firstinspires.ftc.teamcode.wrist.subsystems.WristSubsystem

enum class OpModeType {
    TELEOP, AUTONOMOUS
}
class Robot(val hardwareMap: HardwareMap, val hw: HardwareManager, t: Telemetry, val opModeType: OpModeType = OpModeType.TELEOP) {

    // add multiple telemetry here for dashboard here
    val telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, t)

    val entity = Entity()

    // add subsystems here
    val driveBase = MecanumDriveBase(this)
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