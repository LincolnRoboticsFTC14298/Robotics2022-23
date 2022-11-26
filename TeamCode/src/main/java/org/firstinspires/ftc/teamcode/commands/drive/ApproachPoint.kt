package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.Mecanum

/**
 * Approaches a global point while accepting an input controlling speed or aggressiveness.
 * Driver cannot control rotation, only how fast it moves towards the point.
 * @param targetPoint                   Target point the robot center should be at in global space.
 * @param offsetPose                    Offset vector in robot tangent space. This offset vector
 *                                      will approach the target instead.
 * @param speed                         Controls how fast the robot drives forward towards the point.
 * @param maxTolerableDistance          Maximum distance it can be from the target in m.
 * @param maxTolerableAngleDifference   Maximum angle it can face away from the target in radians.
 */
class ApproachPoint(
    private val mecanum: Mecanum,
    private val targetPoint: () -> Vector2d,
    private val offsetPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val speed: () -> Double,
    private val maxTolerableDistance: Double = 0.005, // m
    private val maxTolerableAngleDifference: Double = 0.05 // radians
) : SequentialCommandGroup() {

    init {
        addCommands(
            ApproachRelativePoint(
                mecanum,
                {
                    val pose = mecanum.getPoseEstimate()
                    val dT =  targetPoint.invoke().minus(pose.vec())
                    dT.rotated(-pose.heading)
                },
                offsetPose,
                speed,
                maxTolerableDistance,
                maxTolerableAngleDifference
            )
        )
        addRequirements(mecanum)
    }

}