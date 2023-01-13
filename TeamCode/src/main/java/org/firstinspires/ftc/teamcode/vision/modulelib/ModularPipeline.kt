package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

/**
 * This pipeline automatically handles initialization and manages the cache.
 * @author Jared Haertel
 */
abstract class ModularPipeline() : OpenCvPipeline() {

    private var endModules = mutableListOf<AbstractPipelineModule<*>>()

    /**
     * Add the end modules to the pipeline for proper management.
     * @param endModules      The end points of the pipeline. Any module you wish
     *                        to get an output from for data or being displayed.
     */
    fun addEndModules(vararg endModules: AbstractPipelineModule<*>) {
        this.endModules.addAll(endModules)
    }

    override fun init(input: Mat) {
        ensureNonEmptyEndModules()
        for (module in endModules) module.init(input)
    }

    protected abstract fun processFrameForCache(input: Mat): Mat

    override fun processFrame(input: Mat): Mat {
        ensureNonEmptyEndModules()
        clearCache()
        return processFrameForCache(input)
    }

    private fun clearCache() {
        for (module in endModules) module.clearCache()
    }

    private fun ensureNonEmptyEndModules() {
        check(endModules.isNotEmpty())
    }

}