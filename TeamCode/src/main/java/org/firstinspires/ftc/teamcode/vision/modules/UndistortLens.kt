package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.calib3d.Calib3d
import org.opencv.core.Mat


class UndistortLens (
    private val inputModule: AbstractPipelineModule<Mat>,
    private val cameraMatrix: Mat,
    private val distCoeffs: Mat
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule)
    }

    override fun init(input: Mat) {
        output = input.clone()
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        Calib3d.undistort(inputModule.processFrame(rawInput), output, cameraMatrix, distCoeffs)
        return output
    }

}