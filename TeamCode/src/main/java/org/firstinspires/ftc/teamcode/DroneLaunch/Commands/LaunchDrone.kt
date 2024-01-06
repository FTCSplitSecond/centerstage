import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.claw.subsystems.DronePositions
import org.firstinspires.ftc.teamcode.claw.subsystems.DroneSubsystem

class LaunchDrone(private val launchDrone : DroneSubsystem) : CommandBase() {
    init {
        addRequirements(launchDrone)
    }

    override fun initialize() {
        launchDrone.position = when (launchDrone.position) {
            DronePositions.LAUNCH -> DronePositions.HELD
            DronePositions.HELD -> DronePositions.LAUNCH
        }

    }

    override fun isFinished(): Boolean {
        return launchDrone.movementShouldBeComplete()
    }
}