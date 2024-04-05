package org.firstinspires.ftc.teamcode.elbow.opmodes

import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.turtles.anchor.component.Component
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.component.stock.Delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import dev.turtles.electriceel.opmode.AnchorOpMode
import dev.turtles.lilypad.Button
import dev.turtles.lilypad.impl.FTCGamepad
import org.apache.commons.math3.util.FastMath.pow
import org.firstinspires.ftc.robotcore.internal.system.Misc
import org.firstinspires.ftc.teamcode.elbow.commands.SetElbowPosition
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowConfig
import org.firstinspires.ftc.teamcode.elbow.subsystems.ElbowPosition
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.AutomaticFeedforwardTuner
import org.firstinspires.ftc.teamcode.roadrunner.util.LoggingUtil
import org.firstinspires.ftc.teamcode.robot.commands.UpdateTelemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMechanism
import org.firstinspires.ftc.teamcode.robot.util.OpModeType
import org.firstinspires.ftc.teamcode.swerve.utils.clamp
import org.firstinspires.ftc.teamcode.telescope.commands.SetTelescopePosition
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopePosition
import java.io.FileNotFoundException
import java.io.PrintWriter
import kotlin.math.sqrt

@TeleOp

class ElbowKvKaEstimate: AnchorOpMode() {
    lateinit var driver: FTCGamepad
    lateinit var robot: Robot
    data class Sample(val time: Double, val power : Double, val angle : Double, val telescopeExtension : Double)

    override fun prerun() {
        robot = Robot(hardwareMap, this.hardwareManager, telemetry, OpModeType.TELEOP)
        driver = FTCGamepad(gamepad1)
        robot.init(this.world)
        +robot.scoringMechanism.setArmState(ScoringMechanism.State.TRAVEL)
    }

    override fun run() {
        robot.elbow.isTelemetryEnabled = true

        driver[Button.Key.DPAD_DOWN] onActivate instant {
            +SetElbowPosition(robot.elbow, ElbowPosition.Adjust(robot.elbow.targetAngle - ElbowConfig.ELBOW_TEST_INCREMENT))
        }
        driver[Button.Key.DPAD_UP] onActivate instant {
            +SetElbowPosition(robot.elbow, ElbowPosition.Adjust(robot.elbow.targetAngle + ElbowConfig.ELBOW_TEST_INCREMENT))
        }

        driver[Button.Key.DPAD_RIGHT] onActivate instant {
            val rx = (robot.telescope.currentExtensionInches + ElbowConfig.ELBOW_TEST_TELESCOPE_INCREMENT).clamp(0.0,
                TelescopeConfig.TELESCOPE_MAX)
            +SetTelescopePosition(robot.telescope, TelescopePosition.Adjust(rx))
        }
        driver[Button.Key.DPAD_LEFT] onActivate instant {
            val rx = (robot.telescope.currentExtensionInches - ElbowConfig.ELBOW_TEST_TELESCOPE_INCREMENT).clamp(0.0,
                TelescopeConfig.TELESCOPE_MAX)
            +SetTelescopePosition(robot.telescope, TelescopePosition.Adjust(rx))
        }

        telemetry.addLine("Press X to run velocity ramp test")
        telemetry.addLine("Press O to run max power test")
        +UpdateTelemetry(robot) { }

        driver[Button.Key.CROSS] onActivate instant { +series(RampVelocity(), Delay(1.0) ,SetElbowPosition(robot.elbow, ElbowPosition.Travel)) }
        driver[Button.Key.CIRCLE] onActivate instant { +series(RampAcceleration(), Delay(1.0), SetElbowPosition(robot.elbow, ElbowPosition.Travel)) }
    }

    val maxAngle = 60.0 // degrees
    val maxPower = 0.25
    val maxAngularVelocity = ElbowConfig.ELBOW_MAX_ANGULAR_VELOCITY

    inner class RampVelocity : Component() {
        private val clock = NanoClock.system()
        private var startTime = clock.seconds()
        private var rampTime = 5.0 //seconds
        private var rampRate = maxPower / rampTime
        private val samples = mutableListOf<Sample>()
        private var isComplete = false

        override fun start() {
            if (robot.elbow.currentAngle > 1.0) isComplete = true
            startTime = clock.seconds()
            samples.clear()
        }
        override fun loop() {
            if (!isComplete) {
                val elapsedTime = clock.seconds() - startTime
                if (robot.elbow.currentAngle >= maxAngle) {
                    isComplete = true
                } else {
                    val power = rampRate * elapsedTime
                    samples.add(
                        Sample(
                            elapsedTime,
                            power,
                            robot.elbow.currentAngle,
                            robot.telescope.currentExtensionInches
                        )
                    )
                    robot.elbow.powerOverride = power
                }
            }
        }
        override fun end(reason: FinishReason) {
            isComplete = true
            robot.elbow.powerOverride = 0.0
            robot.elbow.position = ElbowPosition.Adjust(robot.elbow.currentAngle)
            writeData("elbow_velocity_ramp", samples)

        }
        override fun isComplete() = isComplete
    }
    inner class RampAcceleration : Component() {
            private val clock = NanoClock.system()
            private var startTime = clock.seconds()
            private val samples = mutableListOf<Sample>()

        private var isComplete = false

            override fun start() {
                startTime = clock.seconds()
                samples.clear()
                if(robot.elbow.currentAngle > 0.0) isComplete = true
            }

            override fun loop() {
                if(!isComplete) {
                    val elapsedTime = clock.seconds() - startTime
                    if (robot.elbow.currentAngle >= maxAngle) {
                        isComplete = true
                    } else {
                        val power = maxPower
                        samples.add(Sample(elapsedTime, power, robot.elbow.currentAngle, robot.telescope.currentExtensionInches))
                        robot.elbow.powerOverride = power
                    }
                }
            }

        override fun end(reason: FinishReason) {
            isComplete = true
            robot.elbow.powerOverride = 0.0
            robot.elbow.position = ElbowPosition.Adjust(robot.elbow.currentAngle)
            writeData("elbow_accel_ramp", samples)
        }
        override fun isComplete() = isComplete
    }
    private fun writeData(fileNameBase : String, samples: List<Sample>) {
        val logFile = LoggingUtil.getLogFile(Misc.formatInvariant("${fileNameBase}-%d.csv", System.currentTimeMillis()))
        if (logFile != null) {
            try {
                PrintWriter(logFile).use { pw ->
                    pw.println("time,position,power,telescopeExtension")
                    for (sample in samples)
                        pw.println("${sample.time},${sample.angle},${sample.power},${sample.telescopeExtension}")
                }
            } catch (e: FileNotFoundException) {
                telemetry.addLine("Error writing to file")
            }
        }
    }
}