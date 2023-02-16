package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint

interface Feature {
    fun featureMeasurement(contour: MatOfPoint) : Double
    fun mean() : Double
    fun variance() : Double
    //add index: Int parameter for logging potentially
}