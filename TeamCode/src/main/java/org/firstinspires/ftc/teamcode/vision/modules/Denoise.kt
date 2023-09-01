
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

// Class to denoise an image using morphological operations
class Denoise(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val morphCloseRows: Int,
    private val morphCloseCols: Int,
    private val morphOpenRows: Int,
    private val morphOpenCols: Int
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    // Apply morphological operations to denoise the image
    override fun process(input: Mat): Mat {
        Imgproc.morphologyEx(input, output, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(morphCloseRows, morphCloseCols)))
        Imgproc.morphologyEx(output, output, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(morphOpenRows, morphOpenCols)))
        return output
    }
}
