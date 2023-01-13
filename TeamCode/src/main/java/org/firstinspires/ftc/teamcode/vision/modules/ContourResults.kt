package org.firstinspires.ftc.teamcode.vision.modules

import com.qualcomm.robotcore.hardware.AnalogSensor
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Scorer
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.boundingRect
import java.lang.Math.tan

/**
 * Returns contours that pass the scorers by thresholding a weighted sum.
 * @param scorers Pair of weight and its module scorer
 */
class ContourResults(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val cameraHeight: Double,
    private val FOVX: Double,
    private val FOVY: Double,
    private val cameraPitch: Double = 0.0
) : AbstractPipelineModule<List<ContourResults.AnalysisResult>>() {

    data class AnalysisResult(val point: Point, val angle: Double, val distance: Double)

    init {
        addParentModules(contourModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<AnalysisResult> {
        val results = mutableListOf<AnalysisResult>()
        for (contour in contourModule.processFrame(rawInput)) {
            val boundingBox = boundingRect(contour)
            val pixelX = boundingBox.x+(boundingBox.width/2.0)
            val pixelY = (boundingBox.y+boundingBox.height).toDouble()
            val pixelPoint = Point(pixelX, pixelY) // find pixel location of bottom middle of cone

            // adjust to aiming coordinate space
            val Ax = (pixelPoint.x - (rawInput.size().width/2)) / (rawInput.size().width/2.0)
            val Ay = (pixelPoint.y - (rawInput.size().height/2)) / (rawInput.size().height/2.0)
            val aimingPoint = Point(Ax, Ay)

            // calculate angle and distance
            val pitch = (Ay/2.0)*FOVY
            val yaw = (Ax/2.0)*FOVX
            val distance = -cameraHeight/kotlin.math.tan(cameraPitch + pitch)

            // add to results
            results.add(AnalysisResult(pixelPoint, yaw, distance))
        }
        return results
    }


}