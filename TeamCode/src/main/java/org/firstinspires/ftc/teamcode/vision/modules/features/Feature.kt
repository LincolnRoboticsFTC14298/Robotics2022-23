
package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint

// Interface for feature calculation classes
interface Feature {

    // Method to calculate the feature measurement of a given contour
    fun featureMeasurement(contour: MatOfPoint): Double

    // Method to calculate the mean of the feature measurements
    fun mean(): Double

    // Method to calculate the variance of the feature measurements
    fun variance(): Double

    // Additional methods can be added here, like logging etc.
}
