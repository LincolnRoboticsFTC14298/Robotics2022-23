package org.firstinspires.ftc.teamcode.filters.kalmanFilter

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.ProcessModel

interface KalmanProcessModel : ProcessModel {
    fun getStateTransitionMatrix(previousState: SimpleMatrix, u: SimpleMatrix?, dt: Double): SimpleMatrix
    fun getProcessNoise(dt: Double): SimpleMatrix
}