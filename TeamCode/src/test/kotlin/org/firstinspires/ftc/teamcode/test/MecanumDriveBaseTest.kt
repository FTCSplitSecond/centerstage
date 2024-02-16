package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics
import io.kotest.core.spec.style.ShouldSpec
import io.kotest.matchers.shouldBe
import kotlin.math.sqrt

//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
//import com.arcrobotics.ftclib.geometry.Translation2d
//import io.kotest.core.spec.style.DescribeSpec
//import io.kotest.matchers.shouldBe
//import org.junit.jupiter.api.Assertions.*
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics


class MecanumDriveBaseTest : ShouldSpec({
    context("When comparing the roadrunner kinematics to the FTCLib kinematics") {
        should("with no center of rotation specified, the answers should be the same") {
            // test speed values
            val vx = 1.0
            val vy = 1.0
            val omega = 1.0

            // robot geometry in inches (to match roadrunner)
            val wheelbase = 12.0
            val trackwidth = 12.0
            val x = wheelbase / 2.0
            val y = trackwidth / 2.0
            val m_frontLeftLocation = Translation2d(x, y)
            val m_frontRightLocation = Translation2d(x, -y)
            val m_backLeftLocation = Translation2d(-x, y)
            val m_backRightLocation = Translation2d(-x, -y)

            // calculate Roadrunner kinematics output
            val robotSpeeds = Pose2d(vx, vy, omega)
            val rrWheelSpeeds =
                MecanumKinematics.robotToWheelVelocities(robotSpeeds, wheelbase, trackwidth)

            val chassisSpeeds = ChassisSpeeds(vx* sqrt(2.0), vy* sqrt(2.0), omega* sqrt(2.0))
            val ftclibKinematics = MecanumDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation,
                m_backLeftLocation, m_backRightLocation
            )
            val ftclibWheelSpeeds = ftclibKinematics.toWheelSpeeds(chassisSpeeds)

            // the wheel speeds come in different orders
            val (rrFrontLeft, rrRearLeft, rrRearRight, rrFrontRight) = rrWheelSpeeds

            println("rrFrontLeft: $rrFrontLeft, ftclibFrontLeft: ${ftclibWheelSpeeds.frontLeftMetersPerSecond}")
            println("rrRearLeft: $rrRearLeft, ftclibRearLeft: ${ftclibWheelSpeeds.rearLeftMetersPerSecond}")
            println("rrRearRight: $rrRearRight, ftclibRearRight: ${ftclibWheelSpeeds.rearRightMetersPerSecond}")
            println("rrFrontRight: $rrFrontRight, ftclibFrontRight: ${ftclibWheelSpeeds.frontRightMetersPerSecond}")

            rrFrontLeft shouldBe ftclibWheelSpeeds.frontLeftMetersPerSecond
            rrRearLeft shouldBe ftclibWheelSpeeds.rearLeftMetersPerSecond
            rrRearRight shouldBe ftclibWheelSpeeds.rearRightMetersPerSecond
            rrFrontRight shouldBe ftclibWheelSpeeds.frontRightMetersPerSecond
        }
    }
})