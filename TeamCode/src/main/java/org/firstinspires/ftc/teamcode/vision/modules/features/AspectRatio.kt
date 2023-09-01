
package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect

// Class to calculate the Aspect Ratio feature of a contour
class AspectRatio : Feature {

    // List to store the aspect ratio results
    private val aspectRatioResults = mutableListOf<Double>()

    // Computes the aspect ratio of the given contour
    override fun featureMeasurement(contour: MatOfPoint): Double {
        val boundingBox = boundingRect(contour)
        val aspectRatio = boundingBox.height.toDouble() / boundingBox.width.toDouble()
        aspectRatioResults.add(aspectRatio)
        return aspectRatio
    }
    
    // Other methods like mean(), variance() etc. can be added here
}
