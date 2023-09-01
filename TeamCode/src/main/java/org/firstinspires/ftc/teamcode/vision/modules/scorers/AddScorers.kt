
package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfPoint

// Class to combine scores from multiple scorers
class AddScorers (
    private vararg val scorers: Scorer
) : Scorer {

    // Computes the sum of scores from all scorers
    override fun score(contour: MatOfPoint): Double {
        return scorers.sumOf { scorer -> scorer.score(contour) }
    }
}

// Operator overloading to add scores from two scorers
operator fun Scorer.plus(other: Scorer): Scorer {
    return AddScorers(this, other)
}
