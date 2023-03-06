package org.firstinspires.ftc.teamcode.vision.modules

import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.util.epsilonEquals
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*
import kotlin.math.min
import kotlin.math.tan
import kotlin.math.cos
import kotlin.math.sin


/**
 * Returns contours that pass the scorers by thresholding a weighted sum.
 * @param cameraHeight must be in same units as returned distance
 */
class ContourResults(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val camera: RobotConfig.CameraData,
    private val targetWidth: Double,
    private val targetHeightOffset: Double = 0.0,
    private val pitchDistanceOffset: Double = targetWidth/2.0 // Used incase the center of the object is wanted instead
) : AbstractPipelineModule<List<ContourResults.AnalysisResult>>() {

    data class AnalysisResult(val angle: Double, val distanceByPitch: Double?, val distanceByWidth: Double) {

        fun toVector(useDistanceByWidth: Boolean = false) : Vector2d {
            val distance = chooseDistance(useDistanceByWidth)
            return Vector2d(distance * cos(angle), distance * sin(angle))
        }

        fun distance(other: AnalysisResult, useDistanceByWidth: Boolean = false, useDistanceByWidthForOther: Boolean = false) =
             (toVector(useDistanceByWidth) - other.toVector(useDistanceByWidthForOther)).norm()


        fun sqrDistance(other: AnalysisResult, useDistanceByWidth: Boolean = false, useDistanceByWidthForOther: Boolean = false) =
            (toVector(useDistanceByWidth) - other.toVector(useDistanceByWidthForOther)).sqrNorm()

        private fun chooseDistance(useDistanceByWidth: Boolean) =
            if (useDistanceByWidth) distanceByPitch ?: distanceByWidth else distanceByWidth

        override fun toString() = String.format("Angle: %.1f, Distance by pitch: %.2f, Distance by width: %.2f", Math.toDegrees(angle), distanceByPitch, distanceByWidth)

    }

    init {
        addParentModules(contourModule)
    }

    // TODO Use inverse projection on pixelPoint???
    override fun processFrameForCache(rawInput: Mat): List<AnalysisResult> {
        val contours = contourModule.processFrame(rawInput)
        return List(contours.size) { i ->
            val contour = contours[i]

            val boundingBox = boundingRect(contour)
            val pixelX = boundingBox.x+(boundingBox.width/2.0)
            val pixelY = (boundingBox.y+boundingBox.height).toDouble()
            val pixelPoint = Point(pixelX, pixelY) // find pixel location of bottom middle of contour bounding box

            // adjust to aiming coordinate space
            val Ax = (pixelPoint.x - (rawInput.size().width/2)) / (rawInput.size().width/2.0)
            val Ay = (pixelPoint.y - (rawInput.size().height/2)) / (rawInput.size().height/2.0)

            // calculate angle and distance
            val pitch = (Ay/2.0) * camera.FOVY
            val yaw = (Ax/2.0) * camera.FOVX
            val distanceByPitch = (targetHeightOffset - camera.height) / tan(camera.pitch + pitch) / cos(yaw) + pitchDistanceOffset // TODO Replace w/ inverse projection

            /// New variable
            val contour2f = MatOfPoint2f(*contour.toArray())
            val contourMinAreaRect = minAreaRect(contour2f)
            val measuredWidth = min(contourMinAreaRect.size.width, contourMinAreaRect.size.height)
            val distanceByWidth = targetWidth * camera.fx / measuredWidth

            // add to results
            if (pitch epsilonEquals -camera.FOVY / 2.0) { // not safe to use width for non rectangular outlines
                AnalysisResult(yaw, distanceByPitch, distanceByWidth)
            } else {
                AnalysisResult(yaw, distanceByPitch, distanceByWidth)
            }
        }
    }


}