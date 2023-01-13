package org.firstinspires.ftc.teamcode.filters

import org.apache.commons.math3.linear.RealVector
import org.ejml.simple.SimpleMatrix

interface Filter {
    var stateEstimate: SimpleMatrix
    fun update(z: SimpleMatrix)
}