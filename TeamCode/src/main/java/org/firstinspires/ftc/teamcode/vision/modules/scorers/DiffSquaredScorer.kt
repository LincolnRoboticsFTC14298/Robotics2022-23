package org.firstinspires.ftc.teamcode.vision.modules.scorers


import org.firstinspires.ftc.teamcode.vision.modules.features.Feature
import org.opencv.core.MatOfPoint

class DiffSquaredScorer(
    val feature: Feature,
    private val targetValue: Double,
    private val weight: Double
) : Scorer {

    override fun score(contour: MatOfPoint): Double {
        val measuredValue = feature.featureMeasurement(contour)
        return weight * (measuredValue - targetValue) * (measuredValue - targetValue)
    }
}