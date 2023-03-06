package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.util.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.PIDFController
import kotlin.math.abs

/**
 * Approaches a relative point while accepting an input controlling speed or aggressiveness.
 * Driver cannot control rotation, only how fast it moves towards the point.
 * @param targetPoint                   Target point in robot tangent space.
 * @param offsetPose                    Offset vector in robot tangent space. This offset vector
 *                                      will approach the target instead.
 * @param input                         Vector input in tangent space.
 * @param maxTolerableDistance          Maximum distance it can be from the target in inches.
 * @param maxTolerableAngleDifference   Maximum angle it can face away from the target in radians.
 */
class ApproachRelativePoint(
    private val mecanum: MecanumDrive,
    private val targetPoint: () -> Vector2d?,
    private val offsetPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val input: () -> Twist2d,
    private val maxTolerableDistance: Double = 1.0, // in
    private val maxTolerableAngleDifference: Double = 0.05 // radians
) : SequentialCommandGroup() {

    private val controller = PIDFController(PIDCoefficients(MecanumDrive.HEADING_GAIN, 0.0, MecanumDrive.HEADING_VEL_GAIN))
    init {
        // Automatically handles overflow
        controller.setInputBounds(-Math.PI, Math.PI)

        // Target position is the heading of the offset.
        controller.targetPosition = offsetPose.rot.log()

        val difference = {
            lastTargetAngleSeen = targetPoint.invoke() ?: lastTargetAngleSeen // Spooky ahhh code
            lastTargetAngleSeen - offsetPose.trans
        }

        val distance = { difference.invoke().norm() }

        val localInput = {

            val diff = difference.invoke()

            // Feedforward
            // When the local input is perpendicular to difference, full feedforward is added, when its parallel, the dot product is zero and so no feedforward is done
            // TODO Check
            val thetaFF = -(Rotation2d.exp(-Math.PI / 2) * input.invoke().transVel).dot(diff) / (diff.norm() * diff.norm()) / MecanumDrive.kV

            // Set desired angular velocity to the heading controller output + angular
            // velocity feedforward
            val rot = -controller.update(diff.angleCast().log()) + thetaFF

            Twist2d(input.invoke().transVel, rot)
        }

        addCommands(
            ParallelDeadlineGroup(
                WaitUntilCommand { targetPoint.invoke() != null },
                mecanum.defaultCommand
            ),
            PseudoMotionProfiledDrive(
                mecanum,
                localInput,
                isInputVelocityNormalized = true,
                isInputRotationNormalized = false,
                distance,
                maxTolerableDistance
            )
        )

        addRequirements(mecanum)
    }

    private lateinit var lastTargetAngleSeen: Vector2d

    override fun initialize() {

    }

    override fun isFinished(): Boolean {
        return super.isFinished() && abs(controller.lastError) <= maxTolerableAngleDifference
    }

}