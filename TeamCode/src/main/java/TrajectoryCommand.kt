import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.mecanum.subsystems.MecanumDriveBase
import org.firstinspires.ftc.teamcode.roadrunner.drive.CenterstageMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence

class TrajectoryCommand(
    private val drive: MecanumDriveBase,
    private val trajectory: TrajectorySequence
) :
    CommandBase() {

//    init {
//        addRequirements(drive)
//    }
//
//    override fun initialize() {
//        drive.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun execute() {
//        drive.update()
//    }
//
//    override fun isFinished(): Boolean {
//        return !drive.isBusy
//    }
}