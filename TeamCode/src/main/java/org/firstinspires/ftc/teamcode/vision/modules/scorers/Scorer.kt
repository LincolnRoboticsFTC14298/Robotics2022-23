package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfPoint

interface Scorer {
    fun score(contour: MatOfPoint) : Double
}