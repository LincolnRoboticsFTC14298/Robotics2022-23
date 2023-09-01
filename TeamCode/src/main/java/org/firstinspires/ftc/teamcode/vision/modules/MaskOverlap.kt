
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar

// Class to compute the overlap between two masks
class MaskOverlap(
    private val inputModule1: AbstractPipelineModule<Mat>,
    private val inputModule2: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule1, inputModule2)
    }

    // Compute the overlap using bitwise operations
    override fun process(input: Mat): Mat {
        Core.bitwise_and(inputModule1.output, inputModule2.output, output)
        return output
    }
}
