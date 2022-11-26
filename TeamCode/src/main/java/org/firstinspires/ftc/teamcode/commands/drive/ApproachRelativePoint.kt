package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.subsystems.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.Localizer
import kotlin.math.abs
import kotlin.math.min

/**
 * Approaches a relative point while accepting an input controlling speed or aggressiveness.
 * Driver cannot control rotation, only how fast it moves towards the point.
 * @param targetPoint                   Target point in robot tangent space.
 * @param offsetPose                    Offset vector in robot tangent space. This offset vector
 *                                      will approach the target instead.
 * @param speed                         Controls how fast the robot drives forward towards the point.
 * @param maxTolerableDistance          Maximum distance it can be from the target in m.
 * @param maxTolerableAngleDifference   Maximum angle it can face away from the target in radians.
 * @param timeout                       Timeout after that many seconds if no target provided.
 */
class ApproachRelativePoint(
    private val mecanum: Mecanum,
    private val localizer: Localizer,
    private val targetPoint: () -> Vector2d?,
    private val offsetPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val speed: () -> Double,
    private val maxTolerableDistance: Double = 0.005, // m
    private val maxTolerableAngleDifference: Double = 0.05, // radians
    private val timeout: Double = 2.0
) : CommandBase() {

    init {
        addRequirements(mecanum, localizer)
    }

    private val controller = PIDFController(SampleMecanumDrive.HEADING_PID)

    private var distanceFromTarget: Double = Double.MAX_VALUE

    private var target: Vector2d? = null

    private val timeoutTimer = ElapsedTime()

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
            // Desired movement direction is towards the error
            val inputDir = errorVec.div(distanceFromTarget)
            // Get the currently velocity of the drivetrain
            val v = velocityPose.vec().norm()
            // Distance required to go from current velocity to zero
            val distTillDeaccel = v * v / (2 * DriveConstants.MAX_ACCEL)
            // Calculate maximum allowable magnitude of the input by taking into account need for de-acceleration
            // If the distanceFromTarget <= distTillDeaccel (i.e. the robot should de-accelerate),
            // the fraction will be less than one and is the correct scaling behavior to de-accelerate
            val maxPower = min(distanceFromTarget / distTillDeaccel, 1.0)
            val power = speed.invoke() * maxPower
            // Rotate driveInput by heading to go from tangent space to global space
            val driveInput = inputDir.times(power).rotated(localizer.poseEstimate.heading)

            // Turning //
            // github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpAlignWithPoint.java
            // Set desired angular velocity to the heading controller output
            val turnInput = controller.update(errorVec.angle()) * DriveConstants.kV * DriveConstants.TRACK_WIDTH

            mecanum.setWeightedDrivePower(Pose2d(driveInput, turnInput))
        }
    }

    override fun isFinished(): Boolean {
        return distanceFromTarget <= maxTolerableDistance
                && abs(controller.lastError) <= maxTolerableAngleDifference
                || timeoutTimer.seconds() > timeout
    }

}
