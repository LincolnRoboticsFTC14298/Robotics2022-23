package org.firstinspires.ftc.teamcode.filters.kalmanFilter

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.util.times

class LinearProcessModel(
    private val transitionMatrix: SimpleMatrix,
    private val controlMatrix: SimpleMatrix?,
    private val processNoiseCovariance: SimpleMatrix
) : KalmanProcessModel {

    override fun predictState(
        previousState: SimpleMatrix,
        u: SimpleMatrix?,
        dt: Double
    ): SimpleMatrix {
        var predictedState = transitionMatrix * previousState

        if (u != null && controlMatrix != null) {
            predictedState += controlMatrix*u
        }

        return predictedState
    }

    override fun getStateTransitionMatrix(
        previousState: SimpleMatrix,
        u: SimpleMatrix?,
        dt: Double
    ): SimpleMatrix {
        return transitionMatrix
    }

    override fun getProcessNoise(dt: Double): SimpleMatrix {
        return processNoiseCovariance
    }

}