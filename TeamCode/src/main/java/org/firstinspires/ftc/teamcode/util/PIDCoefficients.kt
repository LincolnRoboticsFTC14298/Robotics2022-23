package org.firstinspires.ftc.teamcode.util

data class PIDCoefficients(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0
)