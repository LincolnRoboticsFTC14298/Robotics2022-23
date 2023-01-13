package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

/**
 * Simple modular pipeline implementation for arbitrary display and output modules.
 * @author Jared Haertel
 */
class DefaultModularPipeline(
    private val displayModule: AbstractPipelineModule<Mat>,
    private vararg val outputModules: AbstractPipelineModule<*>
    ) : ModularPipeline() {

    init {
        addEndModules(displayModule, *outputModules)
    }

    var lastOutput: Array<Any?> = arrayOfNulls(outputModules.size)
        private set

    override fun processFrameForCache(input: Mat): Mat {
        outputModules.forEachIndexed { i, module -> lastOutput[i] = module.processFrame(input) }
        return displayModule.processFrame(input)
    }

}