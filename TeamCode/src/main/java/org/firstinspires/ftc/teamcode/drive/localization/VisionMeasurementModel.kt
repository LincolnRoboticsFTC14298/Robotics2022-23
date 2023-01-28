package org.firstinspires.ftc.teamcode.drive.localization

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.filters.particleFilter.ProbabilisticMeasurementModel
import org.firstinspires.ftc.teamcode.util.matrixToVector
import org.firstinspires.ftc.teamcode.vision.modules.ContourResults.AnalysisResult
import org.opencv.core.Point
import kotlin.math.*

class VisionMeasurementModel : ProbabilisticMeasurementModel {
    override fun calculateProbability(z: SimpleMatrix, state: SimpleMatrix): Double {
        var probability = 1.0

        val z = matrixToList(z)
        val expected = getExpectedObservations(state)
        val pairs = getCorrespondence(z, expected)

        for (pair in pairs) {
            probability *= gaussianProbability(pair.first.angle, pair.second.angle, 0.0)
            probability *= gaussianProbability(pair.first.distance, pair.second.distance, 0.0)
        }

        // TODO adjust for the different number of observation and when len(expected) != len(z)

        return probability
    }

    private fun getExpectedObservations(state: SimpleMatrix): List<AnalysisResult> {

        val polesInSight = mutableListOf<AnalysisResult>()

        for (pole in enumValues<RobotConfig.Pole>().toList()) {
            val diff = pole.vector - matrixToVector(state)
            val localVec = diff.rotated(-state.get(2))

            val yaw = localVec.angle()
            val dist = localVec.norm()
            // TODO Pixel/Projection

            // Only add if pole within camera FOV
            if (abs(yaw) <= RobotConfig.WEBCAM_FOVX/2.0)
                polesInSight.add(AnalysisResult(Point(0.0, 0.0), yaw, dist))
        }

        return polesInSight.toList()
    }

    /**
     * Assumes tight convergence
     * TODO GENERALIZE
     */
    private fun getCorrespondence(z: List<AnalysisResult>, expected: List<AnalysisResult>): List<Pair<AnalysisResult, AnalysisResult>> {
        return List(z.size) { i ->
            val currentObservation = z[i]
            val closestPole = expected.minBy { distanceSqr(currentObservation, it) }
            Pair(currentObservation, closestPole)
        }
    }

    private fun distanceSqr(p1: AnalysisResult, p2: AnalysisResult): Double {
        val a = p1.distance
        val b = p2.distance
        val dtheta = p1.angle - p2.angle
        return a*a + b*b - 2*a*b*cos(dtheta)
    }

    private fun matrixToList(z: SimpleMatrix): List<AnalysisResult> {
        return List(z.numCols()) {i ->
            AnalysisResult(Point(z.get(0, i), z.get(1, i)), z.get(2, i), z.get(3, i))
        }
    }

    private val denom = 1.0 / sqrt(2*PI)
    private fun gaussianProbability(observation: Double, expected: Double, standardDeviation: Double): Double {
        val zScore = (observation - expected) / standardDeviation
        return exp(-zScore * zScore / 2.0) / standardDeviation * denom
    }

}