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
    ) : OpenCvPipeline() {

    var lastOutput: Array<Any?> = arrayOfNulls(outputModules.size)
        private set

    override fun init(input: Mat) {
        displayModule.init(input)
        outputModules.forEach { module -> module.init(input) }
    }

    override fun processFrame(input: Mat): Mat {
        // Clear cache
        displayModule.clearCache()
        for (module in outputModules) module.clearCache()

        // Calculate fresh values
        outputModules.forEachIndexed { i, module -> lastOutput[i] = module.processFrame(input) }
        return displayModule.processFrame(input)
    }

}