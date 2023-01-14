package org.firstinspires.ftc.teamcode.drive.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.RobotConfig.FORWARD_OFFSET
import org.firstinspires.ftc.teamcode.RobotConfig.GEAR_RATIO
import org.firstinspires.ftc.teamcode.RobotConfig.LATERAL_DISTANCE
import org.firstinspires.ftc.teamcode.RobotConfig.TICKS_PER_REV
import org.firstinspires.ftc.teamcode.RobotConfig.WHEEL_RADIUS
import org.firstinspires.ftc.teamcode.RobotConfig.X_MULTIPLIER
import org.firstinspires.ftc.teamcode.RobotConfig.Y_MULTIPLIER
import org.firstinspires.ftc.teamcode.RobotConfig.frontEncoderName
import org.firstinspires.ftc.teamcode.RobotConfig.leftEncoderName
import org.firstinspires.ftc.teamcode.RobotConfig.rightEncoderName
import org.firstinspires.ftc.teamcode.filters.ProcessModel
import org.firstinspires.ftc.teamcode.util.Encoder
import org.firstinspires.ftc.teamcode.util.matrixToPose
import org.firstinspires.ftc.teamcode.util.poseToDoubleArray

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class OdometryLocalizer(
    hwMap: HardwareMap
) : ProcessModel, Localizer {
    private val leftEncoder: Encoder
    private val rightEncoder: Encoder
    private val frontEncoder: Encoder

    init {
        // TODO switch to ftclib encoders
        leftEncoder = Encoder(hwMap.get(DcMotorEx::class.java, leftEncoderName))
        rightEncoder = Encoder(hwMap.get(DcMotorEx::class.java, rightEncoderName))
        frontEncoder = Encoder(hwMap.get(DcMotorEx::class.java, frontEncoderName))

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null

    var wheelDeltas = emptyList<Double>()
    private var lastWheelPositions = emptyList<Double>()

    private val forwardSolver: DecompositionSolver

    init {
        val wheelPoses = listOf(
            Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // left
            Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),  // right
            Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
        )
        require(wheelPoses.size == 3) { "3 wheel positions must be provided" }

        val inverseMatrix = Array2DRowRealMatrix(3, 3)
        for (i in 0..2) {
            val orientationVector = wheelPoses[i].headingVec()
            val positionVector = wheelPoses[i].vec()
            inverseMatrix.setEntry(i, 0, orientationVector.x)
            inverseMatrix.setEntry(i, 1, orientationVector.y)
            inverseMatrix.setEntry(
                i,
                2,
                positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }

        forwardSolver = LUDecomposition(inverseMatrix).solver

        require(forwardSolver.isNonSingular) { "The specified configuration cannot support full localization" }
    }

    fun calculatePoseDelta(wheelDeltas: List<Double>): Pose2d {
        val rawPoseDelta = forwardSolver.solve(
            MatrixUtils.createRealMatrix(
                arrayOf(wheelDeltas.toDoubleArray())
            ).transpose()
        )
        return Pose2d(
            rawPoseDelta.getEntry(0, 0),
            rawPoseDelta.getEntry(1, 0),
            rawPoseDelta.getEntry(2, 0)
        )
    }

    override fun update() {
        val wheelPositions = getWheelPositions()
        if (lastWheelPositions.isNotEmpty()) {
            wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
            val robotPoseDelta = calculatePoseDelta(wheelDeltas)
            _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, robotPoseDelta)
        } else {
            wheelDeltas = listOf(0.0, 0.0, 0.0)
        }

        val wheelVelocities = getWheelVelocities()
        poseVelocity = calculatePoseDelta(wheelVelocities)
        lastWheelPositions = wheelPositions
    }

    /**
     * Used for particle filter.
     */
    override fun predictState(previousState: SimpleMatrix, u: SimpleMatrix?, dt: Double): SimpleMatrix {
        return if (u != null && lastWheelPositions.isNotEmpty()) {
            val pose = matrixToPose(previousState)
            val noisyWheelDeltas = u.ddrm.data.toList()
            val robotPoseDelta = calculatePoseDelta(noisyWheelDeltas)
            val newPose = Kinematics.relativeOdometryUpdate(pose, robotPoseDelta)
            SimpleMatrix(arrayOf(poseToDoubleArray(newPose))).transpose()
        } else {
            previousState
        }
    }

    private fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
        )
    }

    private fun getWheelVelocities(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.correctedVelocity) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.correctedVelocity) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.correctedVelocity) * Y_MULTIPLIER
        )
    }

    private fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }
}