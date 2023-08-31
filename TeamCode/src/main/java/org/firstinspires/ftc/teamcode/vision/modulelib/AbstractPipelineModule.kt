package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat

abstract class AbstractPipelineModule<T> : PipelineModule<T> {

    private var cachedValue: T? = null
    private val parents: MutableList<AbstractPipelineModule<*>> = mutableListOf()

    fun addParentModules(vararg parents: AbstractPipelineModule<*>) {
        this.parents.addAll(parents)
    }

    override fun init(input: Mat) {
        parents.forEach { it.init(input) }
    }

    abstract fun processFrameForCache(input: Mat): T

    override fun processFrame(input: Mat): T {
        return cachedValue ?: processFrameForCache(input).also { cachedValue = it }
    }

    fun clearCache() {
        cachedValue = null
        parents.forEach { it.clearCache() }
    }
}
