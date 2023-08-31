package org.firstinspires.ftc.teamcode.vision.modules

import android.util.Log
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.util.LogFiles.log
import org.firstinspires.ftc.teamcode.util.epsilonEquals
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*
import kotlin.math.*

class ContourResults(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val camera: Vision.Companion.CameraData,
    private val targetWidth: Double,
    private val targetHeightOffset: Double = 0.0,
    pitchDistanceOffset: Double = targetWidth / 2.0
) : AbstractPipelineModule<List<ContourResults.AnalysisResult>>() {

    private val pitchDistanceOffset = pitchDistanceOffset

    data class AnalysisResult(val angle: Double, val distanceByPitch: Double?, val distanceByWidth: Double) {

        fun toVector(useDistanceByWidth: Boolean = false): Vector2d {
            val distance = chooseDistance(useDistanceByWidth)
            return Vector2d(distance * cos(angle), distance * sin(angle))
        }

        fun distance(other: AnalysisResult, useDistanceByWidth: Boolean = false, useDistanceByWidthForOther: Boolean = false) =
            (toVector(useDistanceByWidth) - other.toVector(useDistanceByWidthForOther)).norm()

        fun sqrDistance(other: AnalysisResult, useDistanceByWidth: Boolean = false, useDistanceByWidthForOther: Boolean = false) =
            (toVector(useDistanceByWidth) - other.toVector(useDistanceByWidthForOther)).sqrNorm()

        private fun chooseDistance(useDistanceByWidth: Boolean) =
            distanceByPitch?.takeIf { !useDistanceByWidth } ?: distanceByWidth
    }

    init {
        addParentModules(contourModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<AnalysisResult> {
        return contourModule.processFrame(rawInput).map { contour ->
            val bottom = contour.toList().maxByOrNull { it.y }!!
            calculateDistanceAndAngle(bottom, rawInput)
        }
    }

    private fun calculateDistanceAndAngle(bottom: Point, rawInput: Mat): AnalysisResult {
        val (Ax, Ay) = calculateNormalizedCoordinates(bottom, rawInput.size().width, rawInput.size().height)
        val (pitch, yaw) = calculatePitchAndYaw(Ax, Ay)
        val distanceByPitch = calculateDistanceByPitch(pitch, yaw)?.let { it + pitchDistanceOffset }

        val contour2f = MatOfPoint2f(*contour.toArray())
        val contourMinAreaRect = minAreaRect(contour2f)
        val measuredWidth = contourMinAreaRect.size.width
        val distanceByWidth = targetWidth * camera.fx / measuredWidth

        return AnalysisResult(yaw, distanceByPitch.takeIf { pitch epsilonEquals -camera.FOVY / 2.0 }, distanceByWidth)
    }

    private fun calculateNormalizedCoordinates(bottom: Point, width: Double, height: Double): Pair<Double, Double> {
        val Ax = (bottom.x - width / 2.0) / (width / 2.0)
        val Ay = (height / 2.0 - bottom.y) / (height / 2.0)
        return Ax to Ay
    }

    private fun calculatePitchAndYaw(Ax: Double, Ay: Double): Pair<Double, Double> {
        val pitch = Ay * camera.FOVY / 2.0
        val yaw = Ax * camera.FOVX / 2.0
        return pitch to yaw
    }

    private fun calculateDistanceByPitch(pitch: Double, yaw: Double): Double? {
        return (targetHeightOffset - camera.height) / tan(camera.pitch + pitch) / cos(yaw)
    }
}
