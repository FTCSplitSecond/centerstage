import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.TriggerPositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem

class LaunchDrone(private val launchDrone : DroneSubsystem) : SplitSecondComponent() {

    override fun start() {
        launchDrone.triggerPosition = when (launchDrone.triggerPosition) {
            TriggerPositions.RELEASE -> TriggerPositions.HELD
            TriggerPositions.HELD -> TriggerPositions.RELEASE
            else -> TriggerPositions.RELEASE

        }

    }

    override fun isComplete(): Boolean {
        return launchDrone.movementShouldBeComplete()
    }
}