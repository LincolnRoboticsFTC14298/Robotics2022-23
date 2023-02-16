package org.firstinspires.ftc.teamcode.drive.localization

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.filters.particleFilter.ProbabilisticMeasurementModel
import org.firstinspires.ftc.teamcode.subsystems.Vision.ObservationResult
import org.firstinspires.ftc.teamcode.util.matrixToVector
import kotlin.math.*

class VisionMeasurementModel(private val camera: RobotConfig.CameraData) : ProbabilisticMeasurementModel {
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

    private fun getExpectedObservations(state: SimpleMatrix): List<ObservationResult> {

        val polesInSight = mutableListOf<ObservationResult>()

        for (pole in enumValues<RobotConfig.Pole>().toList()) {
            val diff = pole.vector - matrixToVector(state)
            val localVec = diff.rotated(-state.get(2))

            val yaw = localVec.angle()
            val dist = localVec.norm()
            // TODO Pixel/Projection

            // Only add if pole within camera FOV
            if (abs(yaw) <= camera.FOVX/2.0) // TODO This check fails to account for the offset between camera and robot center
                polesInSight.add(ObservationResult(yaw, dist))
        }

        return polesInSight.toList()
    }

    /**
     * Assumes tight convergence
     * TODO GENERALIZE
     */
    private fun getCorrespondence(z: List<ObservationResult>, expected: List<ObservationResult>): List<Pair<ObservationResult, ObservationResult>> {
        return List(z.size) { i ->
            val currentObservation = z[i]
            val closestPole = expected.minBy { it.distance(currentObservation) }
            Pair(currentObservation, closestPole)
        }
    }

    private fun matrixToList(z: SimpleMatrix): List<ObservationResult> {
        return List(z.numCols()) {i ->
            ObservationResult(z.get(0, i), z.get(1, i))
        }
    }

    private val denom = 1.0 / sqrt(2*PI)
    private fun gaussianProbability(observation: Double, expected: Double, standardDeviation: Double): Double {
        val zScore = (observation - expected) / standardDeviation
        return exp(-zScore * zScore / 2.0) / standardDeviation * denom
    }

}