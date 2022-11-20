package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modulelib.PipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar


class Filter (
    private val inputFrame: AbstractPipelineModule<Mat>,
    private val lowerBound: Scalar,
    private val upperBound: Scalar
    ) : AbstractPipelineModule<Mat>(inputFrame) {

    private lateinit var output: Mat

    override fun init(input: Mat) {
        output = input.clone()
    }

    override fun processFrameForCache(input: Mat): Mat {
        Core.inRange(inputFrame.processFrame(input), lowerBound, upperBound, output)
        return output
    }

}