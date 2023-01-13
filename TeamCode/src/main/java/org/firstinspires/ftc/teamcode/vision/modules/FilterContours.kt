package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Scorer
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc

/**
 * Returns contours that pass the scorers by thresholding a weighted sum.
 * @param scorers Pair of weight and its module scorer
 */
class FilterContours(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val threshold: Double,
    private vararg val scorers: Pair<Double, Scorer>
) : AbstractPipelineModule<List<MatOfPoint>>() {

    init {
        addParentModules(contourModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<MatOfPoint> {
        val contours = contourModule.processFrame(rawInput)
        val finalContours = mutableListOf<MatOfPoint>()
        for (contour in contours) {


            // Check aspect ratio
            val boundingBox = Imgproc.boundingRect(contour)

            var weightedSum = 0.0
            for (scorer in scorers) {
                weightedSum += scorer.first * scorer.second.score(contour)
            }

            if (weightedSum <= threshold) {
                finalContours.add(contour)
            }
        }
        return finalContours
    }


}