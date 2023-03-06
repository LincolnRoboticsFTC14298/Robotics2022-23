package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.Rotation2d
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.filters.particleFilter.ProbabilisticMeasurementModel
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.subsystems.Vision.ObservationResult
import org.firstinspires.ftc.teamcode.util.matrixToVector
import kotlin.math.*

class VisionMeasurementModel(private val camera: Vision.Companion.CameraData) : ProbabilisticMeasurementModel {
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

        for (pole in enumValues<FieldConfig.Pole>().toList()) {
            val diff = pole.vector - matrixToVector(state)
            val localVec = Rotation2d.exp(state.get(2)).inverse() * diff

            val yaw = localVec.angleCast().log()
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
    private fun getCorrespondence(z: List<ObservationResult>, expected: List<ObservationResult>) =
        z.map { observation ->
            val closestPole = expected.minBy { it.sqrDistance(observation) }
            Pair(observation, closestPole)
        }

    private fun matrixToList(z: SimpleMatrix) =
        List(z.numCols()) { i ->
            ObservationResult(z.get(0, i), z.get(1, i))
        }

    private val denom = 1.0 / sqrt(2*PI)
    private fun gaussianProbability(observation: Double, expected: Double, standardDeviation: Double): Double {
        val zScore = (observation - expected) / standardDeviation
        return exp(-zScore * zScore / 2.0) / standardDeviation * denom
    }

}