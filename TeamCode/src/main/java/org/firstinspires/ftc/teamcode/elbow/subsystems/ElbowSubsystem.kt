package org.firstinspires.ftc.teamcode.elbow.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.subsystems.Robot


enum class ElbowPositions{
    INTAKE,
    SCORE
}
class ElbowSubsystem(private val elbowMotor: DcMotorEx, private val telemetry: Telemetry) : SubsystemBase() {
    constructor(robot: Robot):
            this(robot.hardwareMap.get(DcMotorEx::class.java, "elbowMotor"),
                robot.telemetry)


    companion object{
        const val GROUND_POSITION = 0.0
        const val SCORING_POSITION = 500.0
        const val MAX_LIFT_SPEED_UP = 0.4
        const val MAX_LIFT_SPEED_DOWN = 0.4
    }

    fun stopMotor(){
        elbowMotor.power = 0.0
    }

    private var isTelemetryEnabled = false
    override fun periodic() {
        if(isTelemetryEnabled){
            telemetry.addLine("Elbow: Telemetry Enables")
        }
    }

}