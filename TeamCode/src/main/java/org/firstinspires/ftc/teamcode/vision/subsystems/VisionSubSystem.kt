package org.firstinspires.ftc.teamcode.vision.subsystems

import PropDetector
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.turtles.anchor.component.FinishReason
import dev.turtles.anchor.entity.Subsystem
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.processors.PixelStackDetector
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

class VisionSubsystem(private val hardwareMap: HardwareMap, val telemetry: Telemetry): Subsystem() {
    private val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.packageName
    )
    private val webcam = OpenCvCameraFactory.getInstance().createWebcam(
        hardwareMap.get(
            WebcamName::class.java, "webcam1"
        ), cameraMonitorViewId
    )
    val propDetector = PropDetector(telemetry)
    val pixelStackDetector = PixelStackDetector(telemetry)
    init {

    }
    override fun end(reason: FinishReason) {
        TODO("Not yet implemented")
    }

    override fun init() {
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })
        webcam.setPipeline(propDetector)
    }

    fun stopStreaming() {
        webcam.stopStreaming()
    }
    fun switchToStackDetectorPipeline() {
        webcam.setPipeline(pixelStackDetector)
    }
    override fun loop() {
        telemetry.addData("Camera ID", cameraMonitorViewId)
    }

}