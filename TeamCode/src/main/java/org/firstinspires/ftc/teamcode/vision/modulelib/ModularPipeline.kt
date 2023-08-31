package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvPipeline

abstract class ModularPipeline : OpenCvPipeline() {

    private val endModules = mutableListOf<AbstractPipelineModule<*>>()

    fun addEndModules(vararg endModules: AbstractPipelineModule<*>) {
        this.endModules.addAll(endModules)
    }

    override fun init(input: Mat) {
        ensureNonEmptyEndModules()
        endModules.forEach { it.init(input) }
    }

    protected abstract fun processFrameForCache(input: Mat): Mat

    override fun processFrame(input: Mat): Mat {
        ensureNonEmptyEndModules()
        clearCache()
        return processFrameForCache(input)
    }

    private fun clearCache() {
        endModules.forEach { it.clearCache() }
    }

    private fun ensureNonEmptyEndModules() {
        check(endModules.isNotEmpty())
    }
}
