package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.floor

class Mecanum(
    hwMap: HardwareMap,
    vision: Vision,
    localizer: Localizer = OdometryLocalizer(hwMap)
) : SubsystemBase() {

    val drive = SampleMecanumDrive(hwMap, vision, localizer)

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean = false): TrajectoryBuilder {
        return drive.trajectoryBuilder(startPose, reversed)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return drive.trajectoryBuilder(startPose, startHeading)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return drive.trajectorySequenceBuilder(startPose)
    }

    fun turn(angle: Double) {
        drive.turnAsync(angle)
    }

    fun followTrajectory(trajectory: Trajectory) {
        drive.followTrajectoryAsync(trajectory)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        drive.followTrajectorySequence(trajectorySequence)
    }

    fun getLastError(): Pose2d {
        return drive.lastError
    }

    override fun periodic() {
        drive.update()
    }

    fun getPoseEstimate(): Pose2d {
        return drive.poseEstimate
    }

    fun getVelocityEstimate(): Pose2d? {
        return drive.poseVelocity
    }

    fun isBusy(): Boolean {
        return drive.isBusy
    }

    fun setMode(runMode: DcMotor.RunMode) {
        drive.setMode(runMode)
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        drive.setZeroPowerBehavior(zeroPowerBehavior)
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode, coefficients: PIDFCoefficients) {
        drive.setPIDFCoefficients(runMode, coefficients)
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        drive.setWeightedDrivePower(drivePower)
    }

    fun getWheelPositions(): List<Double> {
        return drive.getWheelPositions()
    }

    fun getWheelVelocities(): List<Double>? {
        return drive.getWheelVelocities()
    }

    /**
     * TODO: TEST MULTIPLE ALGORITHMS
     *  Maybe use the velocity vector as well.
     * @return Gets the pole the robot is facing by minimizing the difference of heading.
     */
    fun getFacingPole(): RobotConfig.Pole? {
        val headVec = getPoseEstimate().headingVec()
        val posVec = getPoseEstimate().vec()

        /*
        val minX = floor(posVec.x / RobotConfig.tileSize)
        val minY = floor(posVec.y / RobotConfig.tileSize)

        var highestCosT = -1.0
        var optimalPole: RobotConfig.Pole? = null

        for (x in listOf(minX * RobotConfig.tileSize, (minX+1) * RobotConfig.tileSize)) {
            for (y in listOf(minY * RobotConfig.tileSize, (minY+1) * RobotConfig.tileSize)) {
                val corner = Vector2d(x, y)
                val pole = RobotConfig.Pole.getPole(corner)
                if (pole != null) {
                    val diff = corner.minus(posVec)
                    val dist = diff.norm()
                    val distSignedCosT = diff.dot(headVec) / (dist * dist) // divide by dist twice so that further values have smaller result

                    // Highest dot product means the angle between the corner and heading is lowest
                    if (distSignedCosT > highestCosT) {
                        highestCosT = distSignedCosT
                        optimalPole = pole
                    }
                }
            }
        }
        */

        return enumValues<RobotConfig.Pole>().maxBy {
            val diff = it.vector.minus(posVec)
            val dist = diff.norm()
            diff.dot(headVec) / (dist * dist) // divide by dist twice so that further values have smaller result
        }
    }

    fun getClosestPoleOfType(poleType: RobotConfig.PoleType) : RobotConfig.Pole {
        val poles = RobotConfig.Pole.getPolesOfType(poleType)

        val headVec = getPoseEstimate().headingVec()
        val posVec = getPoseEstimate().vec()

        return poles.maxBy {
            val diff = it.vector.minus(posVec)
            val dist = diff.norm()
            diff.dot(headVec) / (dist * dist) // divide by dist twice so that further values have smaller result
        }
    }
}