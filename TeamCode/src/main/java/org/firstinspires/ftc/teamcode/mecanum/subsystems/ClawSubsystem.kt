package org.firstinspires.ftc.teamcode.mecanum.subsystems

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


    val maximumMovementTime = abs(servo.maximumAngle - servo.minimumAngle) * servoSpeedEstimate / 60.0
    private var estimatedTimeToComplete = 0.0 //seconds

    var angle : Double
    private set (value) {
            servo.angle = value
    }
    get() = servo.angle


    private final var isTelemetryEnabled = false
    override fun periodic(){
        if(isTelemetryEnabled) {
            robot.telemetry.addLine("Claw: Telemetry Enabled")
            robot.telemetry.addData("claw angle", servo.angle)
            robot.telemetry.update()
        }
    }

}