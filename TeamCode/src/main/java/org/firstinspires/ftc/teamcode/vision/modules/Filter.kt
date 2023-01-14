package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar


class Filter (
    private val inputModule: AbstractPipelineModule<Mat>,
    private val lowerBound: Scalar,
    private val upperBound: Scalar
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
        Core.inRange(inputModule.processFrame(rawInput), lowerBound, upperBound, output)
        return output
    }

}