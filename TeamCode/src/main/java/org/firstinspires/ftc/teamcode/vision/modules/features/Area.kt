
package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.contourArea

// Class to calculate the Area feature of a contour
class Area : Feature {

    // List to store the area results
    private val areaResults = mutableListOf<Double>()

    // Computes the area of the given contour
    override fun featureMeasurement(contour: MatOfPoint): Double {
        val area = contourArea(contour)
        areaResults.add(area)
        return area
    }
    
    // Computes the mean of the area measurements
    override fun mean(): Double {
        return areaResults.sum() / areaResults.size
    }
    
    // Other methods like variance() etc. can be added here
}
