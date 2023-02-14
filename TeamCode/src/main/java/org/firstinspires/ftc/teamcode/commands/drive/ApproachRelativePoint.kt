package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * Approaches a relative point while accepting an input controlling speed or aggressiveness.
 * Driver cannot control rotation, only how fast it moves towards the point.
 * @param targetPoint                   Target point in robot tangent space.
 * @param offsetPose                    Offset vector in robot tangent space. This offset vector
 *                                      will approach the target instead.
 * @param speed                         Controls how fast the robot drives forward towards the point.
 * @param maxTolerableDistance          Maximum distance it can be from the target in inches.
 * @param maxTolerableAngleDifference   Maximum angle it can face away from the target in radians.
 * @param timeout                       Timeout after that many seconds if no target provided.
 */
class ApproachRelativePoint(
    private val mecanum: Mecanum,
    private val targetPoint: () -> Vector2d?,
    private val offsetPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val speed: () -> Double,
    private val maxTolerableDistance: Double = 1.0, // in
    private val maxTolerableAngleDifference: Double = 0.05, // radians
    private val timeout: Double = 2.0
) : CommandBase() {

    init {
        addRequirements(mecanum)
    }

    private val controller = PIDFController(SampleMecanumDrive.HEADING_PID)

    private var distanceFromTarget: Double = Double.MAX_VALUE

    private var target: Vector2d? = null

    private val timeoutTimer = ElapsedTime()

    private val loopTimer = ElapsedTime()

    override fun initialize() {
        // Automatically handles overflow
        controller.setInputBounds(-Math.PI, Math.PI)

        // Target position is the heading of the offset.
        controller.targetPosition = offsetPose.heading
    }

    override fun execute() {
        val possibleTarget = targetPoint.invoke()
        if (possibleTarget != null) {
            // Save target in case the targetPoint goes offline
            target = possibleTarget
            // Reset timeout timer because the target is fresh
            timeoutTimer.reset()
        }

        if (target != null) {
            // Calculate error taking into account the offsetPose space
            val errorVec = target!!.minus(offsetPose.headingVec())
            distanceFromTarget = errorVec.norm()

            // Get velocity pose for translation and turning
            val velocityPose = mecanum.getVelocityEstimate()!!

            // Translating //
            // Online Motion Profiling //
            // Desired movement direction is towards the error
            val direction = errorVec.normalize()
            // Get the currently velocity of the drivetrain
            val currentVel = velocityPose.vec().norm()
            // Get the time from the last loop
            val dt = loopTimer.seconds()
            // Calculate the maximum velocity it can drive before needing to stop
            val maxVelToStop = sqrt(2 * MAX_ACCEL * distanceFromTarget)
            // Calculate the maximum and minimum velocity that the drivetrain could possibly travel
            val maxVelFromLast = currentVel + MAX_ACCEL * dt
            val minVelFromLast = currentVel - MAX_ACCEL * dt
            // Choose the minimum possible speed and obtain the max of the physically possible velocity
            val velocity = max(minVelFromLast, minOf(maxVelFromLast, speed.invoke() * MAX_VEL))
            // Rotate driveInput by heading to go from tangent space to global space
            val velVector = (direction * velocity).rotated(mecanum.getPoseEstimate().heading)

            // Turning //
            // github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpAlignWithPoint.java
            // Set desired angular velocity to the heading controller output
            // TODO CHECK
            val omega = controller.update(errorVec.angle()) * DriveConstants.kV * DriveConstants.TRACK_WIDTH

            mecanum.drive.setDriveSignal(DriveSignal(Pose2d(velVector, omega)))
        }

        loopTimer.reset()
    }

    override fun isFinished(): Boolean {
        return distanceFromTarget <= maxTolerableDistance
                && abs(controller.lastError) <= maxTolerableAngleDifference
                || timeoutTimer.seconds() > timeout
    }

}