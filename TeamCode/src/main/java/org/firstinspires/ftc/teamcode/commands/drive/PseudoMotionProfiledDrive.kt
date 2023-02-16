package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * Input in field frame
 */
class PseudoMotionProfiledDrive(
    private val mecanum: Mecanum,
    private val input: () -> Vector2d,
    private val rotation: () -> Double,
    private val distanceFromTarget: () -> Double = { Double.MAX_VALUE },
    private val maxTolerableDistance: Double = 1.0, // in
) : CommandBase() {

    val timer = ElapsedTime()

    init {
        timer.reset()
        addRequirements(mecanum)
    }

    override fun execute() {
        val currentVel = mecanum.getVelocityEstimate()!!.vec().norm()

        val desiredInput = input.invoke()

        // Get the time from the last loop
        val dt = timer.seconds()

        // Calculate the maximum velocity it can drive before needing to stop
        val maxVelToStop = sqrt(2 * DriveConstants.MAX_ACCEL * distanceFromTarget.invoke()) // TODO update to account for tangential driving

        // Calculate the maximum and minimum velocity that the drivetrain could possibly travel
        val maxVelFromLast = currentVel + DriveConstants.MAX_ACCEL * dt
        val minVelFromLast = currentVel - DriveConstants.MAX_ACCEL * dt

        // Choose the minimum possible speed and obtain the max of the physically possible velocity
        val velocity = max(minVelFromLast, minOf(maxVelFromLast, desiredInput.norm() * DriveConstants.MAX_VEL, maxVelToStop))

        // Rotate driveInput by heading to go from tangent space to global space
        val velVector = (desiredInput.normalize() * velocity).rotated(mecanum.getPoseEstimate().heading)

        mecanum.drive.setDriveSignal(DriveSignal(Pose2d(velVector, rotation.invoke())))
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return distanceFromTarget.invoke() <= maxTolerableDistance
    }

}
