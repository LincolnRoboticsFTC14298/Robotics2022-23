package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.subsystems.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder

class Mecanum(
    hwMap: HardwareMap
) : SubsystemBase() {

    private val drive =
        SampleMecanumDrive(hwMap)

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

    fun update() {
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

}