package org.firstinspires.ftc.teamcode.common.component

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive

class TrajectoryFollower(val drive: CenterstageMecanumDrive, val trajectory: Trajectory): SplitSecondComponent() {
    override fun start() {
        drive.followTrajectoryAsync(trajectory)
    }

    override fun isComplete(): Boolean = !drive.isBusy

    override fun end(reason: FinishReason) {
        if (reason != FinishReason.COMPLETED) {
            drive.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
        }
    }
}