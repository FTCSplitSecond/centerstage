import dev.turtles.anchor.component.stock.delay
import dev.turtles.anchor.component.stock.instant
import dev.turtles.anchor.component.stock.series
import org.firstinspires.ftc.teamcode.util.SplitSecondComponent
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.TriggerPositions
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.DroneSubsystem
import org.firstinspires.ftc.teamcode.drone_launcher.Subsystems.PitchPositions

class LaunchDrone(private val launchDrone : DroneSubsystem) : SplitSecondComponent() {

    override fun start() {
        series(
            instant { launchDrone.pitchPosition = PitchPositions.LAUNCH },
            delay(0.25),
            instant {launchDrone.triggerPosition = TriggerPositions.RELEASE},
            delay(0.25),
            instant { launchDrone.pitchPosition = PitchPositions.STOWED },

        )

    }



    override fun isComplete(): Boolean {
        return launchDrone.movementShouldBeComplete()
    }
}