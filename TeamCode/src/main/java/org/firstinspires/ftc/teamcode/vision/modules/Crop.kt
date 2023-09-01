
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Size

// Class to crop an image to a specified region
class Crop(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val rect: Rect
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    // Crop the image to the specified region
    override fun process(input: Mat): Mat {
        output = Mat(input, rect)
        return output
    }
}
