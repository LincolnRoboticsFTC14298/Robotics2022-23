package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.firstinspires.ftc.teamcode.vision.modules.features.Feature
import org.opencv.core.MatOfPoint
import kotlin.math.pow

class DiffSquaredScorer(
    private val feature: Feature,
    private val targetValue: Double,
    private val weight: Double
) : Scorer {
    
    override fun score(contour: MatOfPoint): Double =
        weight * (feature.featureMeasurement(contour) - targetValue).pow(2)
}
