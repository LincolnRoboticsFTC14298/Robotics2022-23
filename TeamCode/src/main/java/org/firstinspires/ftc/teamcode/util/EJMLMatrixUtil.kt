package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.ejml.simple.SimpleMatrix

operator fun SimpleMatrix.times(M: SimpleMatrix): SimpleMatrix = this.mult(M)

operator fun SimpleMatrix.plus(M: SimpleMatrix): SimpleMatrix = this.plus(M)

operator fun SimpleMatrix.minus(M: SimpleMatrix): SimpleMatrix = this.minus(M)

fun vectorToDoubleArray(vector: Vector2d) =
    arrayOf(vector.x, vector.y).toDoubleArray()

fun poseToDoubleArray(pose: Pose2d) =
    arrayOf(pose.x, pose.y, pose.heading).toDoubleArray()

fun matrixToVector(matrix: SimpleMatrix) =
    Vector2d(matrix.get(0), matrix.get(1))

fun matrixToPose(matrix: SimpleMatrix) =
    Pose2d(matrix.get(0), matrix.get(1), matrix.get(2))