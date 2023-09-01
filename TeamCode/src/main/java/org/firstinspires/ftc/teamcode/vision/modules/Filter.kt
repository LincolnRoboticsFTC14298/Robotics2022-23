
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar

// Class to filter an image based on color bounds
class Filter(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val lowerBound: Scalar,
    private val upperBound: Scalar
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule)
    }

    // Apply color filtering to the input image
    override fun process(input: Mat): Mat {
        Core.inRange(input, lowerBound, upperBound, output)
        return output
    }
}
