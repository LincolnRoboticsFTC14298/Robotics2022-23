package org.firstinspires.ftc.teamcode.filters.kalmanFilter

import org.ejml.simple.SimpleMatrix

interface KalmanMeasurementModel {
    fun predictObservation(state: SimpleMatrix): SimpleMatrix
    fun getObservationMatrix(state: SimpleMatrix): SimpleMatrix
    fun getObservationNoise(): SimpleMatrix
}