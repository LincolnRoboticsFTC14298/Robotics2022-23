package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.abs

/**
 * @return Normalized vector.
 */
fun Vector2d.normalize(): Vector2d = this.div(this.norm())

const val EPSILON = 1e-6
infix fun Double.epsilonEquals(other: Double) = abs(this - other) < EPSILON