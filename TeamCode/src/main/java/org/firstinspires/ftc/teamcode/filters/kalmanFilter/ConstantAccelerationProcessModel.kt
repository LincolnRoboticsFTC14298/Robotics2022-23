package org.firstinspires.ftc.teamcode.filters.kalmanFilter

import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.ejml.simple.SimpleMatrix

class ConstantAccelerationProcessModel : KalmanProcessModel {

    override fun predictState(
        previousState: SimpleMatrix,
        u: SimpleMatrix?,
        dt: Double
    ): SimpleMatrix {
        return getStateTransitionMatrix(previousState, u, dt).mult(previousState) + (u?:SimpleMatrix(3,1))
    }

    override fun getStateTransitionMatrix(
        previousState: SimpleMatrix,
        u: SimpleMatrix?,
        dt: Double
    ): SimpleMatrix {
        return SimpleMatrix(
            arrayOf(
                doubleArrayOf(1.0, dt, 0.5 * dt * dt),
                doubleArrayOf(0.0, 1.0, dt),
                doubleArrayOf(0.0, 0.0, 1.0)
            )
        )
    }

    override fun getProcessNoise(dt: Double): SimpleMatrix {
        val dt4 = dt * dt * dt * dt
        val dt3 = dt * dt * dt
        val dt2 = dt * dt
        return SimpleMatrix(
            arrayOf(
                doubleArrayOf(dt4 / 4.0, dt3 / 2.0, dt2 / 2.0),
                doubleArrayOf(dt3 / 2.0, dt2, dt),
                doubleArrayOf(dt2 / 2.0, dt, 1.0)
            )
        ).scale(0.1)
    }
}