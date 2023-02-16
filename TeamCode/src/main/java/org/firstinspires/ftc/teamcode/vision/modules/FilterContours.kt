package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Scorer
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint

/**
 * Returns contours that pass the scorers by thresholding a weighted sum.
 * @param scorers Pair of weight and its module scorer
 */
class FilterContours(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val threshold: Double,
    private val scorer: Scorer
) : AbstractPipelineModule<List<MatOfPoint>>() {

    init {
        addParentModules(contourModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<MatOfPoint> {
        val contours = contourModule.processFrame(rawInput)
        val finalContours = mutableListOf<MatOfPoint>()
        for (contour in contours) {

            if (scorer.score(contour) <= threshold) {
                finalContours.add(contour)
            }
        }
        return finalContours
    }


}