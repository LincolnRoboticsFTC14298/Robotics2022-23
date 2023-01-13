package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar


class MaskOverlap (
    private val inputModule1: AbstractPipelineModule<Mat>,
    private val inputModule2: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule1, inputModule2)
    }

    override fun init(input: Mat) {
        super.init(input)
        output = input.clone()
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        Core.bitwise_and(inputModule1.processFrame(rawInput), inputModule2.processFrame(rawInput), output)
        return output
    }

}