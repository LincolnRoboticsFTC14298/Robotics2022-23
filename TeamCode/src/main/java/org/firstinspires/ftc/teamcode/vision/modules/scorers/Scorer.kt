
package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfPoint

// Interface for scoring classes
interface Scorer {

    // Method to calculate the score of a given contour
    fun score(contour: MatOfPoint): Double

    // Additional methods like logging can be added here
}
