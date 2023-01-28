package org.firstinspires.ftc.teamcode.filters.kalmanFilter

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.Filter
import org.firstinspires.ftc.teamcode.util.times

class KalmanFilter(
    private val processModel: KalmanProcessModel,
    private val measurementModel: KalmanMeasurementModel,
    initialStateEstimate: SimpleMatrix,
    private var covariance: SimpleMatrix
) : Filter<SimpleMatrix, SimpleMatrix> {

    override var stateEstimate: SimpleMatrix = initialStateEstimate

    lateinit var K: SimpleMatrix
        private set

    fun predict(u: SimpleMatrix?, dt: Double) {
        // Prediction covariance
        val F = processModel.getStateTransitionMatrix(stateEstimate, u, dt)

        // Predict Estimate
        stateEstimate = processModel.predictState(stateEstimate, u, dt)

        covariance = F * covariance * F.transpose() + processModel.getProcessNoise(dt)
    }

    override fun update(z: SimpleMatrix) {
        // y[k]
        val innovation = z - measurementModel.predictObservation(stateEstimate)

        // S[k] = H[k] P[k|k-1] HT[k] + R[k]
        val H = measurementModel.getObservationMatrix(stateEstimate)
        val innovationCovariance = H * covariance * H.transpose() + measurementModel.getObservationNoise()

        // Kalman gain
        // K[k] = P[k/k-1] HT[K] S[K]-1
        // TODO: possible numerical errors may occur
        K = covariance * H.transpose() * innovationCovariance.invert()

        // Update state estimate
        // x[k|k] = x[k|k-1] + K[k] y[k]
        stateEstimate += K * innovation

        // Update covariance estimate
        // P[k|k] = (I - K[k] H[k]) P[k|k-1]
        // P[k|k] = P[k|k-1] - K[k] H[k] P[k|k-1]
        covariance -= K * H * covariance;
    }
}