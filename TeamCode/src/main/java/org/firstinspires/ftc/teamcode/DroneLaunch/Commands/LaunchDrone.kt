import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.DroneLaunch.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.DroneLaunch.Subsystems.DroneSubsystem

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