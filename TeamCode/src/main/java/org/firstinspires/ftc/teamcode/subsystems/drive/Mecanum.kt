package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.floor

class Mecanum(
    hwMap: HardwareMap
) : SubsystemBase() {

    val drive = SampleMecanumDrive(hwMap)

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
        val tileVec = getPoseEstimate().vec().div(RobotConfig.tileSize)

        val minX = floor(tileVec.x / RobotConfig.tileSize)
        val minY = floor(tileVec.y / RobotConfig.tileSize)

        val highestCosT = -1.0
        var optimalPole: RobotConfig.Pole? = null

        for (x in listOf(minX * RobotConfig.tileSize, (minX+1) * RobotConfig.tileSize)) {
            for (y in listOf(minY * RobotConfig.tileSize, (minY+1) * RobotConfig.tileSize)) {
                val corner = Vector2d(x, y)
                val pole = RobotConfig.Pole.getPole(corner)
                if (pole != null) {
                    val diff = corner.minus(tileVec).normalize()
                    val cosT = diff.dot(headVec)

                    // Highest dot product means the angle between the corner and heading is lowest
                    if (cosT > highestCosT) {
                        optimalPole = pole
                    }
                }
            }
        }

        return optimalPole
    }
}