package org.firstinspires.ftc.teamcode.swerve.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import org.firstinspires.ftc.teamcode.swerve.subsystems.SwerveDriveConfiguration

class SwerveModuleDriveMotor(private val driveMotor: MotorEx) : SubsystemBase() {
    init {
        // register so tha the command scheduler can call periodic() where we update the drive motor
        register()

        // set up the drive motor
        // TODO: Tune the velocity PID coefficients, and move them to the SwerveDriveConfiguration.
        //       We may need a different one for each swerveModule because each has a different friction.
        driveMotor.setRunMode(Motor.RunMode.VelocityControl)
        driveMotor.setVeloCoefficients(0.05, 0.01, 0.31); // might want to move this to the SwerveDriveConfiguration
        driveMotor.setDistancePerPulse(SwerveDriveConfiguration.wheelCircumference * SwerveDriveConfiguration.wheelRevsPerMotorRev / driveMotor.cpr)
    }

    var velocity : Double
        get() = driveMotor.velocity
        set(value) { driveMotor.velocity = value}

    var inverted : Boolean
        get() = driveMotor.inverted
        set(value) { driveMotor.inverted = value}

    override fun periodic() {
        runDriveMotorPID()
    }
    private fun runDriveMotorPID() {
        // TODO: determine if we should use the built-in motor PID controller or the external PIDController class
        // right now, we are using the motor's built-in PID controller
        // we could also use the external PIDController class from the ftclib
        // if so, we would do an external PID Loop here.
        // The internal PID controller is not as flexible as the ftclib PIDController
        // and it runs at 20Hz (50ms).  With and external PID some teams can get 80Hz (12.5ms)
    }
}