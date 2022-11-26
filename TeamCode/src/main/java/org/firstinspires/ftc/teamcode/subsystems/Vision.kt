package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.ConePipeline
import org.firstinspires.ftc.teamcode.vision.PolePipeline
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvInternalCamera
import org.openftc.easyopencv.OpenCvPipeline

/**
 * Manages all the pipelines and cameras.
 */
class Vision(
    hwMap: HardwareMap
) : SubsystemBase() {

    // TODO: Setup webcam and phonecam hardware objects
    // TODO: Setup pipelines

    enum class RearPipeline(pipeline: OpenCvPipeline) {
        //APRIL_TAG(aprilTagPipeline),
        //POLE(polePipeline)
    }

    init {
        name = "Vision Subsystem"

        // Open cameras asynchronously and load the pipelines
        // TODO

    }



    /**
     * Starts streaming the front camera.
     */
    fun startStreamingFrontCamera() {
        // TODO
        // use .startStreaming()
    }

    /**
     * Stops streaming the front camera.
     */
    fun stopStreamingFrontCamera() {
        // TODO
        // use .stopStreaming()
    }

    /**
     * Starts streaming the rear camera.
     */
    fun startStreamingRearCamera() {
        // TODO
        // use .startStreaming()
    }

    /**
     * Stops streaming the rear camera.
     */
    fun stopStreamingRearCamera() {
        // TODO
        // use .stopStreaming()
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
    fun getConeRelativePosition(): Vector2d? {
        // TODO
        return Vector2d(0.0, 0.0)
    }

    /**
     * @return List of relative position of the poles in the robot tangent space,
     *         sometimes using cones instead of the poles themselves as reference points.
     */
    fun getPoleRelativePositions(): List<Vector2d> {
        // TODO
        return emptyList()
    }

}