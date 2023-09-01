
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

// Class to convert the color space of an image
class ColorConverter(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val colorMode: Int = Imgproc.COLOR_RGB2Lab
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule)
    }

    // Converts the color space of the input image
    override fun process(input: Mat): Mat {
        Imgproc.cvtColor(input, output, colorMode)
        return output
    }
}
