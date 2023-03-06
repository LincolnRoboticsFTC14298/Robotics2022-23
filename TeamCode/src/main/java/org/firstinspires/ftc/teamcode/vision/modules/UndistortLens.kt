package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.calib3d.Calib3d
import org.opencv.core.Mat


class UndistortLens (
    private val inputModule: AbstractPipelineModule<Mat>,
    private val camera: Vision.Companion.CameraData
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule)
    }

    override fun init(input: Mat) {
        super.init(input)
        output = input.clone()
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        Calib3d.undistort(inputModule.processFrame(rawInput), output, camera.getCameraMatrix(), camera.distCoeffs)
        return output
    }

}