package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import dev.turtles.anchor.component.FinishReason
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive

/**
 * Follows a roadrunner trajectory until completion
 */
class TrajectoryFollower(val drive: CenterstageMecanumDrive, val trajectory: Trajectory): SplitSecondComponent() {
    override fun start() {
        drive.followTrajectoryAsync(trajectory)
    }

    override fun isComplete() = !drive.isBusy

    override fun end(reason: FinishReason) {
        if (reason != FinishReason.COMPLETED)
            drive.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
    }
}
