package org.firstinspires.ftc.teamcode.vision.subsystems

import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.ftc24789.Robot
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvWebcam

class AprilTags(val robot: Robot) : SubsystemBase() {
    private val cameraMonitorViewId = robot.hardwareMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId",
        "id",
        robot.hardwareMap.appContext.packageName
    )
    private val cameraIds: IntArray = OpenCvCameraFactory.getInstance()
        .splitLayoutForMultipleViewports(
            cameraMonitorViewId,
            1,
            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY
        )
    private val webcam: OpenCvWebcam = OpenCvCameraFactory.getInstance()
        .createWebcam(robot.hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraIds[0])
    init {
        this.register()
    }
    override fun periodic() {
        //update()
    }


}