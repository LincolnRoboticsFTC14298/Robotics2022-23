package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.ejml.simple.SimpleMatrix

operator fun SimpleMatrix.times(M: SimpleMatrix): SimpleMatrix = this.mult(M)

operator fun SimpleMatrix.plus(M: SimpleMatrix): SimpleMatrix = this.plus(M)

operator fun SimpleMatrix.minus(M: SimpleMatrix): SimpleMatrix = this.minus(M)

operator fun DoubleArray.plus(doubleArray: DoubleArray) : DoubleArray = this.zip(doubleArray).map{ it.first + it.second}.toDoubleArray()

operator fun DoubleArray.minus(doubleArray: DoubleArray) : DoubleArray = this.zip(doubleArray).map{ it.first - it.second}.toDoubleArray()

fun arrayToColumnMatrix(array: DoubleArray) = SimpleMatrix(arrayOf(array)).transpose()
fun arrayToRowMatrix(array: DoubleArray) = SimpleMatrix(arrayOf(array))


fun vectorToMatrix(vector: Vector2d) =
    SimpleMatrix(arrayOf(doubleArrayOf(vector.x, vector.y))).transpose()

fun poseToMatrix(pose: Pose2d) =
    SimpleMatrix(arrayOf(doubleArrayOf(pose.x, pose.y, pose.heading))).transpose()


fun matrixToVector(matrix: SimpleMatrix) =
    Vector2d(matrix.get(0), matrix.get(1))

fun matrixToPose(matrix: SimpleMatrix) =
    Pose2d(matrix.get(0), matrix.get(1), matrix.get(2))