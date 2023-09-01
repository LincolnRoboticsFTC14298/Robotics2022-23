
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar

// Class to combine multiple masks into a single mask
class CombineMasks(
    private val inputModule1: AbstractPipelineModule<Mat>,
    private val inputModule2: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule1, inputModule2)
    }

    // Combine the masks using bitwise operations
    override fun process(input: Mat): Mat {
        Core.bitwise_and(inputModule1.output, inputModule2.output, output)
        return output
    }
}
