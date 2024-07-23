package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.turtles.anchor.entity.Entity
import dev.turtles.anchor.world.World
import dev.turtles.electriceel.wrapper.HardwareManager
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.types.Alliance
import org.firstinspires.ftc.teamcode.common.types.OpModeType
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem
import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystem.telescope.TelescopeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.elbow.ElbowSubsystem
import org.firstinspires.ftc.teamcode.subsystem.wrist.WristSubsystem
import kotlin.math.PI

/**
 * FTC #24789 CenterStage Bot
 */
class OffseasonBot(
    val hwMap: HardwareMap,
    val hw: HardwareManager,
    val opModeType: OpModeType,
    val startPose: Pose2d,
    t: Telemetry,
) {
    var driverStationOffset = when (alliance) {
        Alliance.RED -> -PI / 2
        Alliance.BLUE -> PI / 2
    }

    init {
        instance = this
    }

    val telemetry = MultipleTelemetry(t, FtcDashboard.getInstance().telemetry)

    val robot = Entity()

    val drivetrain: MecanumSubsystem = MecanumSubsystem(this)
    val telescope: TelescopeSubsystem = TelescopeSubsystem(this, hw)
    val claw: ClawSubsystem = ClawSubsystem(this)
    val elbow: ElbowSubsystem = ElbowSubsystem(this)
    val wrist: WristSubsystem = WristSubsystem(this, hw)
    val drone: DroneSubsystem = DroneSubsystem(hw)

    val deposit: DepositSubsystem = DepositSubsystem(claw, wrist, telescope, elbow)

    val elbowEncoder = hw.motor("bR").encoder
    val telescopeEncoder = hw.motor("fR").encoder

    fun init(world: World) {
        robot.add(drivetrain)
        robot.add(telescope)
        robot.add(claw)
        robot.add(elbow)
        robot.add(wrist)
        robot.add(drone)

        world.add(robot)
    }

    val isAuto: Boolean
        get() = opModeType == OpModeType.AUTO

    companion object {
        var lastKnownAutoPose = Pose2d(0.0, 0.0, 0.0)
        var alliance: Alliance = Alliance.RED

        private lateinit var instance: OffseasonBot
            private set
    }
}