package org.firstinspires.ftc.teamcode.test.elbow

import io.kotest.core.spec.style.ShouldSpec
import io.kotest.matchers.doubles.plusOrMinus
import io.kotest.matchers.shouldBe
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowSubsystem

data class ElbowFeedForwardTestVector(val theta: Double, val omega: Double, val alpha: Double, val extension : Double, val expectedPower: Double)
class ElbowTests :  ShouldSpec({
    context("When calculating the feedforward values for the elbow subsystem") {
        should("the answers should match the expected values from the Excel sheet") {

            val testVectors = listOf(
                ElbowFeedForwardTestVector(0.0,0.0,0.0,0.0,0.048195 ),
                ElbowFeedForwardTestVector(90.0,0.0,0.0,0.0,0.0 ),
                ElbowFeedForwardTestVector(0.0,0.0,0.0,20.0,0.197648 )
            )
            val tolerance = 0.0001

            for(test in testVectors) {
                val jt = ElbowSubsystem.getMoment(test.extension)
                val power = ElbowSubsystem.getGravityFeedforward(test.theta, jt)
                power shouldBe (test.expectedPower plusOrMinus tolerance)
            }
        }
        should("Calculated Kv values should match empirical data") {
            val empiricalKv = 0.0026 // measured via curve fit in Nom Power / (deg/sec)
            val omega = 30.0
            val alpha = 0.0
            val j0 = ElbowSubsystem.getMoment(0.0)
            val tolerance = 0.001
            val kvPower = ElbowSubsystem.getVAFeedForwardPower(omega, alpha, j0)
            val kv = kvPower / omega
            kv shouldBe (empiricalKv plusOrMinus tolerance)
        }
    }
})