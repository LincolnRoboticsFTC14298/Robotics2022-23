package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfPoint

interface Scorer {
    fun score(contour: MatOfPoint): Double
    fun scoreWithLogging(contour: MatOfPoint, index: Int): Double = score(contour).also {
        println("Scoring contour at index $index: $it")
    }
}
