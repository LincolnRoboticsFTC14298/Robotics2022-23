package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.singleConeToJunctionMaxDistance
import org.firstinspires.ftc.teamcode.RobotConfig.stackToPoleMaxDistance
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.GeneralPipeline
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera
import org.openftc.easyopencv.OpenCvPipeline

/**
 * Manages all the pipelines and cameras.
 */
class Vision(
    hwMap: HardwareMap,
    startPipeline: FrontPipeline = FrontPipeline.GENERAL_PIPELINE,
    private val telemetry: Telemetry? = null
) : SubsystemBase() {

    val cameraMonitorViewId: Int = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName())
    val webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
    val phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)

    private val dashboard = FtcDashboard.getInstance()

    enum class AprilTagResult(var id: Int) {
        PARK_LEFT(5),
        PARK_MIDDLE(15),
        PARK_RIGHT(9);

        companion object {
            fun find(id: Int): AprilTagResult? = AprilTagResult.values().find { it.id == id }
        }
    }

    enum class FrontPipeline(var pipeline: OpenCvPipeline) {
        APRIL_TAG(AprilTagDetectionPipeline(1.18,  RobotConfig.CameraData.LOGITECH_C920)),
        GENERAL_PIPELINE(GeneralPipeline(GeneralPipeline.DisplayMode.ALL_CONTOURS, RobotConfig.CameraData.LOGITECH_C920, null))
    }

    val phoneCamPipeline = GeneralPipeline(GeneralPipeline.DisplayMode.ALL_CONTOURS, RobotConfig.CameraData.PHONECAM, telemetry)

    init {
        name = "Vision Subsystem"

        (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).telemetry = telemetry

        phoneCam.setPipeline(phoneCamPipeline)

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

        webCam.setPipeline(startPipeline.pipeline)

        webCam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                startStreamingFrontCamera()
            }

            override fun onError(errorCode: Int) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        })
    }

    var numFramesWithoutDetection = 0

    private val DECIMATION_HIGH = 3f
    private val DECIMATION_LOW = 2f
    private val THRESHOLD_HIGH_DECIMATION_RANGE_FEET = 3.0f
    private val THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4
    fun updateAprilTag() : AprilTagResult? {
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        val aprilTagDetectionPipeline = (FrontPipeline.APRIL_TAG.pipeline as AprilTagDetectionPipeline)
        val detections: ArrayList<AprilTagDetection> = aprilTagDetectionPipeline.detectionsUpdate

        // If there's been a new frame...
        if (detections != null) {

            // If we don't see any tags
            if (detections.size == 0) {
                numFramesWithoutDetection++

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW)
                }
            } else {
                numFramesWithoutDetection = 0

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections[0].pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_FEET) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH)
                }
                return AprilTagResult.find(detections.minBy{it.pose.z}.id)
            }
        }

        return null
    }



    /**
     * Starts streaming the front camera.
     */
    fun startStreamingFrontCamera() {
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        dashboard.startCameraStream(webCam, 10.0)
    }

    /**
     * Stops streaming the front camera.
     */
    fun stopStreamingFrontCamera() {
        webCam.stopStreaming()
        dashboard.stopCameraStream()
    }

    /**
     * Starts streaming the rear camera.
     */
    fun startStreamingRearCamera() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        dashboard.startCameraStream(phoneCam, 10.0)
    }

    /**
     * Stops streaming the rear camera.
     */
    fun stopStreamingRearCamera() {
        phoneCam.stopStreaming()
        dashboard.startCameraStream(webCam, 10.0)
    }

    /**
     * Sets the rear pipeline.
     * @param pipeline     From the [RearPipeline] pipeline options.
     */
    fun setFrontPipeline(pipeline: FrontPipeline) {
        webCam.setPipeline(pipeline.pipeline)
    }


    data class ObservationResult(val angle: Double, val distance: Double) {

        companion object {
            fun vector(vector: Vector2d) : ObservationResult {
               return ObservationResult(vector.angle(), vector.norm())
            }
        }
        fun toVector() = Vector2d.polar(distance, angle)

        fun distance(other: ObservationResult) : Double {
            return toVector().distTo(other.toVector())
        }
    }

    /**
     * @return List of pixel info for landmarks from pipeline
     * TODO: Include tall stacks not next to poles
     */
    fun getCameraSpaceLandmarkInfo(): List<ObservationResult> {

        val poles = (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).poleResults
        val stacks = (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).stackResults.toMutableList()

        val landmarks = mutableListOf<ObservationResult>()

        poles.forEach { pole ->
            val closestStack = stacks.minByOrNull { it.distance(pole, useDistanceByWidthForOther = true) } // Find closest stack

            if (closestStack != null && closestStack.distance(pole) < stackToPoleMaxDistance) { // If sufficiently close, simply take the average between them
                val observationVector = closestStack.toVector()

                landmarks.add(ObservationResult.vector(observationVector))
                stacks.remove(closestStack) // Remove once identified as a pair
            } else {
                landmarks.add(ObservationResult(pole.angle, pole.distanceByPitch ?: pole.distanceByWidth))
            }
        }

        return landmarks
    }

    fun getLandmarkInfo(): List<ObservationResult> {
        val cameraLandmarks = getCameraSpaceLandmarkInfo()
        return List(cameraLandmarks.size) { i ->
            val cameraLandmark = cameraLandmarks[i]
            val robotLandmark = cameraLandmark.toVector() + RobotConfig.CameraData.LOGITECH_C920.relativePosition
            ObservationResult.vector(robotLandmark)
        }
    }

    /**
     * Returns pole position in tangent space taking into account cones
     */
    fun getClosestPolePosition(): Vector2d? {
        return getLandmarkInfo().minByOrNull { it.distance }?.toVector()
    }

    fun getClosestPoleAngle(): Double? {
        return getClosestPolePosition()?.angle()
    }
    fun getClosestConePosition(): Vector2d? {
        val cones = phoneCamPipeline.singleConeResults.toMutableList()
        val poles = phoneCamPipeline.poleResults.toMutableList()
        val junctions = enumValues<RobotConfig.Junction>()

        cones.forEachIndexed { i, cone ->
            val closestPole = poles.minByOrNull { it.distance(cone) }
            if (closestPole != null &&
                closestPole.distance(cone) < singleConeToJunctionMaxDistance) {
                cones.removeAt(i)
                poles.remove(closestPole)
            } else {
                val closestJunction = junctions.minBy { it.vector.distTo(cone.toVector()) }
                if (closestJunction.vector.distTo(cone.toVector()) < singleConeToJunctionMaxDistance) {
                    cones.removeAt(i)
                }
            }
        }

        val closestCone = cones.minByOrNull { it.distanceByPitch ?: Double.MAX_VALUE }
        return closestCone?.toVector()?.plus(RobotConfig.CameraData.PHONECAM.relativePosition)
    }

    /**
     * @return Coordinate of the closest cone in the robot tangent space.
     */
    fun getClosestConeAngle(): Double? {
        return getClosestConePosition()?.angle()
    }

}