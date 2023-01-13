package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat

/**
 * Uses lambda for simple operations.
 * @param T                  Type of output.
 * @param onProcessFrame     Lambda expression to perform with input.
 * @param parents            Dependent modules. Used to properly clear cached values.
 * @author Jared Haertel
 */
open class SimpleModule<T>(
    private val onProcessFrame: (input: Mat) -> T,
    vararg parents: AbstractPipelineModule<T>) : AbstractPipelineModule<T>() {

    init {
        addParentModules(*parents)
    }

    override fun processFrameForCache(input: Mat): T {
        return onProcessFrame.invoke(input)
    }

}