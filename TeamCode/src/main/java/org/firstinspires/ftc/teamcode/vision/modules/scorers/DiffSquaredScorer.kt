
package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.firstinspires.ftc.teamcode.vision.modules.features.Feature
import org.opencv.core.MatOfPoint

// Class to calculate the score based on squared difference
class DiffSquaredScorer(
    val feature: Feature,
    private val targetValue: Double,
    private val weight: Double
) : Scorer {

    // Computes the score using the squared difference formula
    override fun score(contour: MatOfPoint): Double {
        val measuredValue = feature.featureMeasurement(contour)
        return weight * (measuredValue - targetValue).let { it * it }
    }
}
