//package org.firstinspires.ftc.teamcode.robot.opmodes
//
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.arcrobotics.ftclib.command.CommandOpMode
//import com.arcrobotics.ftclib.command.ParallelCommandGroup
//import com.arcrobotics.ftclib.command.SequentialCommandGroup
//import com.arcrobotics.ftclib.command.WaitCommand
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import org.firstinspires.ftc.teamcode.claw.commands.CloseBothClaw
//import org.firstinspires.ftc.teamcode.claw.commands.OpenBothClaw
//import org.firstinspires.ftc.teamcode.robot.commands.IncreasePixelLevel
//import org.firstinspires.ftc.teamcode.robot.commands.MoveToDeposit
//import org.firstinspires.ftc.teamcode.robot.commands.MoveToExtendedIntake
//import org.firstinspires.ftc.teamcode.robot.commands.MoveToPlace
//import org.firstinspires.ftc.teamcode.robot.commands.MoveToPosition
//import org.firstinspires.ftc.teamcode.robot.commands.MoveToTravel
//import org.firstinspires.ftc.teamcode.robot.subsystems.OpModeType
//import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
//import kotlin.math.PI
//
//@Autonomous
//class RedPlaceAuto: CommandOpMode() {
//    override fun initialize() {
//        val robot = Robot(hardwareMap, telemetry, OpModeType.AUTONOMOUS)
//
//        val startPose = Pose2d(0.0, 0.0, 0.0)
//        robot.driveBase.poseEstimate = startPose
//        robot.elbow.isEnabled = true
//
//
//        val autoCommands = SequentialCommandGroup(
//            CloseBothClaw(robot.leftclaw, robot.rightClaw),
//            MoveToPosition(robot, Pose2d(48.0, 0.0, 0.0)),
//            MoveToPosition(robot, Pose2d(48.0, 0.0, PI/2)),
//            MoveToPosition(robot, Pose2d(48.0, -72.0, PI/2)),
//            ParallelCommandGroup(
//                MoveToPosition(robot, Pose2d(24.0, -72.0, PI/2)),
//                MoveToDeposit(robot),
//            ),
//            MoveToPosition(robot, Pose2d(24.0, -78.0, PI/2)),
//            ParallelCommandGroup(
//                SequentialCommandGroup(
//                    OpenBothClaw(robot.leftclaw, robot.rightClaw),
//                    MoveToPlace(robot),
//                    ParallelCommandGroup(
//                        CloseBothClaw(robot.leftclaw, robot.rightClaw),
//                        MoveToTravel(robot)),
//                    MoveToPosition(robot, Pose2d(48.0, -86.0 , PI/2))))
//
//
//
//
//        )
//        schedule(autoCommands)
//    }
//}