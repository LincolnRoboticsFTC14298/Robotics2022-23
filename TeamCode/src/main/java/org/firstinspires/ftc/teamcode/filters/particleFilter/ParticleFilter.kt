package org.firstinspires.ftc.teamcode.filters.particleFilter

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.Filter
import org.firstinspires.ftc.teamcode.filters.ProcessModel
import java.util.*

class ParticleFilter(
    private val numberOfParticles: Int,
    private val motionProcessModel: ProcessModel,
    private val motionNoiseStandardDeviations: SimpleMatrix,
    private val probabilisticMeasurementModel: ProbabilisticMeasurementModel
) : Filter<SimpleMatrix, SimpleMatrix> {

    override var stateEstimate: SimpleMatrix = SimpleMatrix(0, 0)
        get() = getAverage()

    lateinit var particles: Array<SimpleMatrix>

    private val random = Random()

    /**
     * Initialize particles with a simple multivariate gaussian distribution.
     * Assumes independence between the states.
     */
    fun initialize(mean: SimpleMatrix, standardDeviation: SimpleMatrix) {
        particles = Array(numberOfParticles) {
            generateGaussianNoiseMatrix(mean, standardDeviation)
        }
    }

    /**
     * Adds noise to the control input vector of the model to predict the next state.
     * TODO: Noise should probably be moved to the prediction model
     */
    fun predict(u: SimpleMatrix, dt: Double) {
        for ((index, particle) in particles.withIndex()) {
            // Generate noise to interject into control vector
            val noise = u.elementMult(generateGaussianNoiseMatrix(motionNoiseStandardDeviations))

            // Predict motion of noisy control vector
            particles[index] = motionProcessModel.predictState(particle, u+noise, dt)
        }
    }

    /**
     * @param z Each column represents a separate measurement of the parameters
     */
    override fun update(z: SimpleMatrix) {
        // Calculate probability of measurements occurring given the particle position
        var totalWeight = 0.0
        val weights = DoubleArray(numberOfParticles) { i ->
            val weight = probabilisticMeasurementModel.calculateProbability(z, particles[i])
            totalWeight += weight
            weight
        }

        // Resample particles
        resampleParticles(weights, totalWeight)
    }

    /**
     * Low variance sampling
     * O(n) :)
     */
    private fun resampleParticles(weights: DoubleArray, totalWeight: Double) : Array<SimpleMatrix> {
        val randomStart = random.nextDouble() / numberOfParticles.toDouble()
        var currentCumulativeWeight = weights[0]
        var currentWeightIndex = 0
        return Array(numberOfParticles) { m ->
            val u = randomStart + m / numberOfParticles.toDouble()
            while (u*totalWeight > currentCumulativeWeight) { // March until in the indicator u is in the correct weight box
                currentWeightIndex += 1
                currentCumulativeWeight += weights[currentWeightIndex]
            }
            particles[currentWeightIndex]
        }
    }

    private fun getAverage() : SimpleMatrix {
        var totalState = SimpleMatrix(particles[0].numRows(), particles[0].numCols())
        for (particle in particles) {
            totalState += particle
        }
        return totalState.divide(numberOfParticles.toDouble())
    }

    private fun generateGaussianNoiseMatrix(mean: SimpleMatrix, standardDeviations: SimpleMatrix) : SimpleMatrix {
        val mat = Array(standardDeviations.numRows()) {
            DoubleArray(standardDeviations.numCols()) {
                random.nextGaussian()
            }
        }

        val gaussianVector = SimpleMatrix(mat)
        return mean + standardDeviations.elementMult(gaussianVector)
    }

    private fun generateGaussianNoiseMatrix(standardDeviations: SimpleMatrix) : SimpleMatrix {
        return generateGaussianNoiseMatrix(SimpleMatrix(standardDeviations.numRows(), standardDeviations.numCols()), standardDeviations)
    }

}