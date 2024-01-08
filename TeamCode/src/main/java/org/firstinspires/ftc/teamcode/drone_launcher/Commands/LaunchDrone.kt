import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.SplitSecondComponent
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DronePositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem

class LaunchDrone(private val launchDrone : DroneSubsystem) : SplitSecondComponent() {

    override fun start() {
        launchDrone.position = when (launchDrone.position) {
            DronePositions.LAUNCH -> DronePositions.HELD
            DronePositions.HELD -> DronePositions.LAUNCH
        }

    }

    override fun isComplete(): Boolean {
        return launchDrone.movementShouldBeComplete()
    }
}