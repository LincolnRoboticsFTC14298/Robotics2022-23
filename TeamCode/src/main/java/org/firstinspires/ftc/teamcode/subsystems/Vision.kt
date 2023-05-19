package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.FieldConfig.coneDiameter
import org.firstinspires.ftc.teamcode.FieldConfig.poleDiameter
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.GeneralPipeline
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin



/**
 * Manages all the pipelines and cameras.
 */
@Config
class Vision(
    hwMap: HardwareMap,
    startingPipeline: FrontPipeline = FrontPipeline.GENERAL_PIPELINE,
    private val telemetry: Telemetry? = null
) : SubsystemBase() {

    var cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.packageName
    )

    var viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY)
    val webCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName::class.java, "Webcam 1"), viewportContainerIds[0])
    val phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[1])

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
        APRIL_TAG(AprilTagDetectionPipeline(1.18,  CameraData.LOGITECH_C920)),
        GENERAL_PIPELINE(GeneralPipeline(GeneralPipeline.DisplayMode.ALL_CONTOURS, CameraData.LOGITECH_C920, null))
    }

    val phoneCamPipeline = GeneralPipeline(GeneralPipeline.DisplayMode.ALL_CONTOURS, CameraData.PHONECAM, telemetry)

    init {
        name = "Vision Subsystem"

        (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).telemetry = telemetry

        // Open cameras asynchronously and load the pipelines
        phoneCam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                phoneCam.setPipeline(phoneCamPipeline)
            }

            override fun onError(errorCode: Int) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        })


        webCam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                webCam.setPipeline(startingPipeline.pipeline)
                webCam.showFpsMeterOnViewport(true)
                webCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)
                startStreamingFrontCamera()
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
        webCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT) // TODO Check
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
                return AprilTagResult.find(detections.minBy{ it.pose.z }.id)
            }
        }

        return null
    }

    data class ObservationResult(val angle: Double, val distance: Double) {

        companion object {
            fun fromVector(vector: Vector2d) : ObservationResult {
               return ObservationResult(vector.angleCast().log(), vector.norm())
            }
        }
        fun toVector() = Vector2d(distance * cos(angle), distance * sin(angle))

        fun distance(other: ObservationResult) = (toVector() - other.toVector()).norm()

        fun sqrDistance(other: ObservationResult) = (toVector() - other.toVector()).sqrNorm()

        operator fun plus(vector: Vector2d) = fromVector(vector + toVector())

        override fun toString() = String.format("Angle: %.1f, Distance: %.2f", Math.toDegrees(angle), distance)
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

                landmarks.add(ObservationResult.fromVector(observationVector))
                stacks.remove(closestStack) // Remove once identified as a pair
            } else {
                landmarks.add(ObservationResult(pole.angle, pole.distanceByPitch ?: pole.distanceByWidth))
            }
        }

        return landmarks
    }

    fun getLandmarkInfo(): List<ObservationResult> = getCameraSpaceLandmarkInfo().map{ it + CameraData.LOGITECH_C920.relativePosition }

    /**
     * Returns pole position in tangent space taking into account cones
     */
    fun getClosestPole() = getLandmarkInfo().minByOrNull { it.distance }

    fun getClosestCone(pose: Pose2d? = null, useWidth: Boolean = false): ObservationResult? {
        val cones = phoneCamPipeline.singleConeResults.toMutableList()
        val poles = phoneCamPipeline.poleResults.toMutableList()
        val junctions = enumValues<FieldConfig.Junction>()

        cones.forEachIndexed { i, cone ->
            val closestPole = poles.minByOrNull { it.distance(cone) }

            // Remove if on pole
            if (closestPole != null &&
                closestPole.distance(cone) < singleConeToJunctionMaxDistance) {
                cones.removeAt(i)
                poles.remove(closestPole)
            }
            else if (pose != null) {
                // Remove if on junction
                val coneFieldFrame = pose * cone.toVector()
                val closestJunction = junctions.minBy { (it.vector - coneFieldFrame).sqrNorm() }
                if ((closestJunction.vector - coneFieldFrame).norm() < singleConeToJunctionMaxDistance) {
                    cones.removeAt(i)
                }
            }
        }

        // Get closest one
        val closestCone = cones.minByOrNull { it.distanceByPitch ?: if(useWidth) it.distanceByWidth else Double.MAX_VALUE }

        // Turn into observation result
        val closestConeObservation = closestCone?.let { ObservationResult(it.angle, it.distanceByPitch ?: it.distanceByWidth) }

        // Transform by the relative phone camera position
        return closestConeObservation?.plus(CameraData.PHONECAM.relativePosition)
    }

    private fun getRawConeStacks() : List<ObservationResult> {
        val stacks = mutableListOf<ObservationResult>()

        phoneCamPipeline.stackResults.forEach { stack ->
            if(stack.distanceByPitch != null) stacks.add(ObservationResult(stack.angle, stack.distanceByPitch))
        }

        (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).stackResults.forEach { stack ->
            if(stack.distanceByPitch != null) stacks.add(ObservationResult(stack.angle, stack.distanceByPitch))
        }

        return stacks
    }

    private fun getRawPoles() : List<ObservationResult> {
        val poles = mutableListOf<ObservationResult>()

        phoneCamPipeline.poleResults.forEach { pole ->
            if (pole.distanceByPitch != null && abs(pole.distanceByPitch - pole.distanceByWidth) < 3.0 ) { // TODO make 3.0 a parameter
                poles.add(ObservationResult(pole.angle, pole.distanceByPitch))
            } else {
                poles.add(ObservationResult(pole.angle, pole.distanceByWidth))
            }
        }

        (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).poleResults.forEach { pole ->
            if (pole.distanceByPitch != null && abs(pole.distanceByPitch - pole.distanceByWidth) < 3.0 ) {
                poles.add(ObservationResult(pole.angle, pole.distanceByPitch))
            } else {
                poles.add(ObservationResult(pole.angle, pole.distanceByWidth))
            }
        }

        return poles
    }

    fun fetchTelemetry(telemetry: Telemetry, pose: Pose2d? = null) {
        telemetry.addLine("Processed data")
        telemetry.addData("Closest cone", getClosestCone(pose)?.toString())
        telemetry.addData("Closest cone using width", "Angle: 0.1f", getClosestCone(pose, useWidth = true)?.angle)
        telemetry.addData("Closest pole", getClosestPole()?.toString())
        telemetry.addData("Landmarks", getLandmarkInfo())

        telemetry.addLine()
        telemetry.addLine("Raw data")
        telemetry.addData("Stacks", (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).stackResults)
        telemetry.addData("Poles", (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).poleResults)
        telemetry.addData("Single cones", (FrontPipeline.GENERAL_PIPELINE.pipeline as GeneralPipeline).singleConeResults)
    }

    fun drawObservations(canvas: Canvas, pose: Pose2d, showAll: Boolean = false) {

        // Draw the landmarks
        val landmarks = getLandmarkInfo()
        canvas.setStroke("#F9A801")
        for (landmark in landmarks) {
            drawObservationResult(canvas, landmark, pose, poleDiameter / 2.0, true)
        }

        // Draw the closest cone
        val cone = getClosestCone(pose, false)
        if (cone != null) {
            canvas.setStroke("#253368")
            drawObservationResult(canvas, cone, pose, coneDiameter / 2.0, true)
        }

        if (showAll) {
            val poles = getRawPoles()
            canvas.setStroke("#EB5E31")
            for (pole in poles) {
                drawObservationResult(canvas, pole, pose, poleDiameter / 2.0, false)
            }

            val stacks = getRawConeStacks()
            canvas.setStroke("#2A4161")
            for (stack in stacks) {
                drawObservationResult(canvas, stack, pose, coneDiameter / 2.0, false)
            }
        }

    }

    private fun drawObservationResult(canvas: Canvas, observation: ObservationResult, pose: Pose2d, radius: Double, fill: Boolean = true) {
        val (x, y) = pose.trans.plus(pose.rot.inverse().times(observation.toVector()))
        if (fill) canvas.fillCircle(x, y, radius)
        else canvas.strokeCircle(x, y, radius)
    }

    companion object {
        enum class CameraData(val pitch: Double, val height: Double, val relativePosition: Vector2d, val FOVX: Double, val FOVY: Double, val fx: Double, val fy: Double, val cx: Double, val cy: Double, val distCoeffs: MatOfDouble) {
            PHONECAM(0.0, 2.0, Vector2d(-5.0, 0.0),
                Math.toRadians(60.0),
                Math.toRadians(60.0), 0.0, 0.0, 0.0, 0.0, MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0)
            ),
            LOGITECH_C920(0.0, 6.0, Vector2d(4.5, 0.0),
                Math.toRadians(70.42),
                Math.toRadians(43.3), 1.44943054e+3, 1.44934063e+3, 9.37759430e+2, 5.34866814e+2, MatOfDouble(0.07622862, -0.41153656, -0.00089351, 0.00219123, 0.57699695)
            );

            fun getCameraMatrix(): Mat {
                val cameraMat = Mat(3, 3, CvType.CV_64FC1)
                val cameraData = doubleArrayOf(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
                cameraMat.put(0, 0, *cameraData)
                return cameraMat
            }
        }

        const val stackToPoleMaxDistance = 5.0
        const val visionToPoleMaxDistance = 5.0 // Difference between vision observation and pole location to be considered the same
        const val singleConeToJunctionMaxDistance = 4.0
    }

}