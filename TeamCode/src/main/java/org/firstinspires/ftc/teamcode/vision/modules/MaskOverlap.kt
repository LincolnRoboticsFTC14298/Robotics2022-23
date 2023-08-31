package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat

class MaskOverlap(
    private val inputModule1: AbstractPipelineModule<Mat>,
    private val inputModule2: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<Mat>() {

    init {
        addParentModules(inputModule1, inputModule2)
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        return Mat().apply {
            val inputFrame1 = inputModule1.processFrame(rawInput)
            val inputFrame2 = inputModule2.processFrame(rawInput)
            org.opencv.core.Core.bitwise_and(inputFrame1, inputFrame2, this)
        }
    }
}
