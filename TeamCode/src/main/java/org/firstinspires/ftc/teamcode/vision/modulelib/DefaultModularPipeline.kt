package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

class DefaultModularPipeline(
    private val displayModule: AbstractPipelineModule<Mat>,
    private vararg val outputModules: AbstractPipelineModule<*>
) : ModularPipeline() {

    init {
        addEndModules(displayModule, *outputModules)
    }

    private var lastOutput: Array<Any?> = arrayOfNulls(outputModules.size)

    override fun processFrameForCache(input: Mat): Mat {
        lastOutput = outputModules.map { it.processFrame(input) }.toTypedArray()
        return displayModule.processFrame(input)
    }
}
