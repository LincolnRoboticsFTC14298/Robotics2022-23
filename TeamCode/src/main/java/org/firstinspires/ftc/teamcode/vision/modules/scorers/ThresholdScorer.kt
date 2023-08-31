package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.firstinspires.ftc.teamcode.vision.modules.features.Feature
import org.opencv.core.MatOfPoint

class ThresholdScorer(
    private val feature: Feature,
    private val range: Pair<Double, Double>,
    private val weight: Double
) : Scorer {
    
    override fun score(contour: MatOfPoint): Double =
        if (feature.featureMeasurement(contour) in range.first..range.second) 0.0 else weight
}
