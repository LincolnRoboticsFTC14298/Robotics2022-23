package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfPoint

class AddScorers(private vararg val scorers: Scorer) : Scorer {
    override fun score(contour: MatOfPoint): Double = scorers.sumOf { it.score(contour) }
}

operator fun Scorer.plus(other: Scorer): Scorer = AddScorers(this, other)
