package org.firstinspires.ftc.teamcode.robot.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d


fun Vector2d.adjustForAlliance(alliance: Alliance): Vector2d {
    return when(alliance) {
        Alliance.RED -> Vector2d(this.x, -this.y)
        Alliance.BLUE -> Vector2d(this.x, this.y)
    }
}
fun Pose2d.adjustForAlliance(alliance: Alliance): Pose2d {
    return Pose2d(this.vec().adjustForAlliance(alliance), this.heading)
}