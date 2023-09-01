
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc

// Class to convert a contour to a binary mask
class ContourToMask(
    private val inputModule: AbstractPipelineModule<MatOfPoint>,
    private val imageSize: Pair<Int, Int>
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    // Convert the contour to a mask
    override fun process(input: MatOfPoint): Mat {
        Imgproc.drawContours(output, listOf(input), -1, Scalar(255.0, 255.0, 255.0), -1)
        return output
    }
}
