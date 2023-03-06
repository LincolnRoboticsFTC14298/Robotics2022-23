package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive

/**
 * Approaches a global point while accepting an input controlling speed or aggressiveness.
 * Driver cannot control rotation, only how fast it moves towards the point.
 * @param targetPoint                   Target point the robot center should be at in global space.
 * @param offsetPose                    Offset vector in robot tangent space. This offset vector
 *                                      will approach the target instead.
 * @param input                         Input in tangent space
 * @param maxTolerableDistance          Maximum distance it can be from the target in inches.
 * @param maxTolerableAngleDifference   Maximum angle it can face away from the target in radians.
 */
class ApproachPoint(
    mecanum: MecanumDrive,
    targetPoint: () -> Vector2d?,
    offsetPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    input: () -> Twist2d,
    maxTolerableDistance: Double = 1.0, // in
    maxTolerableAngleDifference: Double = 0.05 // radians
) : SequentialCommandGroup() {

    init {
        addCommands(
            ApproachRelativePoint(
                mecanum,
                {
                    val target = targetPoint.invoke()
                    if (target != null ) {
                        mecanum.pose.inverse() * target
                    } else {
                        null
                    }
                },
                offsetPose,
                input,
                maxTolerableDistance,
                maxTolerableAngleDifference
            )
        )
        addRequirements(mecanum)
    }

}