package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar


class Filter (
    private val inputFrame: AbstractPipelineModule<Mat>,
    private val lowerBound: Scalar,
    private val upperBound: Scalar
    ) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputFrame)
    }

    override fun init(input: Mat) {
        output = input.clone()
    }

    override fun processFrameForCache(input: Mat): Mat {
        Core.inRange(inputFrame.processFrame(input), lowerBound, upperBound, output)
        return output
    }

}