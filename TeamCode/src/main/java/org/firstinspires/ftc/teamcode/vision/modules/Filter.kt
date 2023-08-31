package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.Scalar

class Filter(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val lowerBound: Scalar,
    private val upperBound: Scalar
) : AbstractPipelineModule<Mat>() {

    init {
        addParentModules(inputModule)
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        return Mat().apply {
            val inputFrame = inputModule.processFrame(rawInput)
            org.opencv.core.Core.inRange(inputFrame, lowerBound, upperBound, this)
        }
    }
}
