package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.calib3d.Calib3d
import org.opencv.core.Mat

class UndistortLens(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val camera: Vision.Companion.CameraData
) : AbstractPipelineModule<Mat>() {

    init {
        addParentModules(inputModule)
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        return Mat().apply {
            val inputFrame = inputModule.processFrame(rawInput)
            Calib3d.undistort(inputFrame, this, camera.getCameraMatrix(), camera.distCoeffs)
        }
    }
}
