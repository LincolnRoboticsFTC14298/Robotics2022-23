package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint

interface Feature {
    fun featureMeasurement(contour: MatOfPoint): Double
    fun mean(): Double
    fun variance(): Double

    // Adding a default method for logging with an index parameter
    fun log(index: Int = 0) {
        println("Feature log at index $index: Mean = ${mean()}, Variance = ${variance()}")
    }
}
