package org.firstinspires.ftc.teamcode.claw.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot
import org.firstinspires.ftc.teamcode.utilities.AxonServo
import kotlin.math.*

enum class ClawPositions{
    OPEN,
    CLOSED
}

class ClawSubsystem(val robot: Robot, val servo: AxonServo): SubsystemBase() {

    init {
        register()

    }

    val servoSpeedEstimate = 0.09

    var position:ClawPositions = ClawPositions.CLOSED
        set(value) {
            val previousAngle = angle
            angle  = getAngleForPosition(value)
            val angleChange = abs(angle - previousAngle)
            estimatedTimeToComplete = (angleChange * servoSpeedEstimate / 60.0).coerceAtMost(maximumMovementTime)
            field = value
        }

    val maximumMovementTime = abs(servo.maximumAngle - servo.minimumAngle) * servoSpeedEstimate / 60.0
    private var estimatedTimeToComplete = 0.0 //seconds

    var angle : Double
    private set (value) {
            servo.angle = value
    }
    get() = servo.angle

    private fun getAngleForPosition(position : ClawPositions) : Double = when (position) {
        ClawPositions.OPEN ->  0.0
        ClawPositions.CLOSED ->  0.0
    }

    private var isTelemetryEnabled = false
    override fun periodic(){
        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Claw: Telemetry Enabled")
            robot.telemetry.addData("claw angle", servo.angle)
            robot.telemetry.addData("claw position", position)
            robot.telemetry.update()
        }
    }

}