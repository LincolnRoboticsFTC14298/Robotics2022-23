package org.firstinspires.ftc.teamcode.filters.kalmanFilter

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.util.times

class LinearMeasurementModel(
    private val observationMatrix: SimpleMatrix,
    private val observationNoise: SimpleMatrix,
): KalmanMeasurementModel {

    override fun predictObservation(state: SimpleMatrix): SimpleMatrix {
        return observationMatrix * state
    }

    override fun getObservationMatrix(state: SimpleMatrix): SimpleMatrix {
        return observationMatrix
    }

    override fun getObservationNoise(): SimpleMatrix {
        return observationNoise
    }

}