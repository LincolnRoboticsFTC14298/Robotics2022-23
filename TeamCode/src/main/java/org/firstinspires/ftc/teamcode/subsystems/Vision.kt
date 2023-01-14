package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.RobotConfig.phoneCamHeight
import org.firstinspires.ftc.teamcode.RobotConfig.webcamHeight
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.GeneralConePipeline
import org.firstinspires.ftc.teamcode.vision.PolePipeline
import org.opencv.core.Point
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera
import org.openftc.easyopencv.OpenCvPipeline

/**
 * Manages all the pipelines and cameras.
 */
class Vision(
    hwMap: HardwareMap
) : SubsystemBase() {

    val cameraMonitorViewId: Int = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName())
    val webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
    val phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)

    enum class RearPipeline(var pipeline: OpenCvPipeline) {
        APRIL_TAG(AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506)),
        POLE(PolePipeline(PolePipeline.DisplayMode.ALL_CONTOURS, 70.42, 43.3, webcamHeight))
    }

    val conePipeline = GeneralConePipeline(GeneralConePipeline.DisplayMode.ALL_CONTOURS, 67.0, 52.9, phoneCamHeight)

    init {
        name = "Vision Subsystem"

        // Open cameras asynchronously and load the pipelines
        phoneCam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {

            }

            override fun onError(errorCode: Int) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        })
    }



    /**
     * Starts streaming the front camera.
     */
    fun startStreamingFrontCamera() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
    }

    /**
     * Stops streaming the front camera.
     */
    fun stopStreamingFrontCamera() {
        phoneCam.stopStreaming()
    }

    /**
     * Starts streaming the rear camera.
     */
    fun startStreamingRearCamera() {
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
    }

    /**
     * Stops streaming the rear camera.
     */
    fun stopStreamingRearCamera() {
        webCam.stopStreaming()
    }

    /**
     * Sets the rear pipeline.
     * @param pipeline     From the [RearPipeline] pipeline options.
     */
    fun setRearPipeline(pipeline: RearPipeline) {
        // backCamera.setPipeline(pipeline.pipeline)
    }

    /**
     * @return Coordinate of the closest cone in the robot tangent space.
     */
    fun getConeAngle(): Double? {
        val results = conePipeline.singleConeResults
        if (results.isNotEmpty()) {
            var closestResult = results[0]
            var smallestDistance = closestResult.distance
            for (result in results) {
                //iterate through find closest result
                if (result.distance<smallestDistance){
                    smallestDistance=result.distance
                    closestResult=result
                }
            }
            return closestResult.angle
        }
        return null
    }

    fun getPoleAngle(): Double? {
        val results = (RearPipeline.POLE.pipeline as PolePipeline).poleResults
        if (results.isNotEmpty()) {
            var closestResult = results[0]
            var smallestDistance = closestResult.distance
            for (result in results) {
                //iterate through find closest result
                if (result.distance<smallestDistance){
                    smallestDistance=result.distance
                    closestResult=result
                }
            }
            return closestResult.angle
        }
        return null
    }

    /**
     * @return List of pixel info for landmarks form pipeline
     */
    fun getLandmarkInfo(): List<Point> {
        TODO("Implement")
    }

    /**
     * @return Returns global position from pixel space
     */
    fun pixelToGlobalPosition(pixel: Point, realHeight: Double, position: Pose2d) : Vector2d {
        TODO("Implement")
    }

}