package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat

/**
 * Automatically caches values to avoid redundant calculations.
 * @param T                  Type of output.
 * @param parents            Dependent modules. Used to properly clear cached values.
 * @author Jared Haertel
 */
abstract class AbstractPipelineModule<T>(private vararg val parents: AbstractPipelineModule<T>) : PipelineModule<T> {

    private var cachedValue: T? = null

    override fun init(input: Mat) {

    }

    abstract fun processFrameForCache(input: Mat): T

    override fun processFrame(input: Mat): T {
        if (cachedValue != null) return cachedValue!!
        cachedValue = processFrameForCache(input)
        return cachedValue!!
    }

    /**
     * Clears cache in preparation for fresh values.
     */
    fun clearCache() {
        cachedValue = null
        for (parent in parents) parent.clearCache() // Redundant calls if overlap, negligible on performance
    }
}