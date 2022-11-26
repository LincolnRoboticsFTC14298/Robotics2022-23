package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorEx
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
import org.firstinspires.ftc.teamcode.util.Encoder

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
class StandardTrackingWheelLocalizer(hwMap: HardwareMap) : ThreeTrackingWheelLocalizer(
    listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // left
        Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),  // right
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
    )
) {
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

    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.correctedVelocity) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.correctedVelocity) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.correctedVelocity) * Y_MULTIPLIER
        )
    }

    companion object {
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}