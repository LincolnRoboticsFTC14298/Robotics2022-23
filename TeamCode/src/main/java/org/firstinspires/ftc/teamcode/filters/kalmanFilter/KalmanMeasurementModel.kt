package org.firstinspires.ftc.teamcode.filters.kalmanFilter

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.MeasurementModel

interface KalmanMeasurementModel : MeasurementModel {
    fun getObservationMatrix(state: SimpleMatrix): SimpleMatrix
    fun getObservationNoise(): SimpleMatrix
}