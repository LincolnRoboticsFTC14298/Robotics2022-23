
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc

// Class to find contours in an image
class Contours(
    private val inputModule: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<List<MatOfPoint>>() {

    private var output = mutableListOf<MatOfPoint>()

    // Finds contours in the input image
    override fun process(input: Mat): List<MatOfPoint> {
        Imgproc.findContours(input, output, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
        return output
    }
}
