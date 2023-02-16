package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.drive.localization.MecanumMonteCarloLocalizer


/**
 * Manual joystick control of drivetrain.
 */
class JoystickDrive(
    mecanum: Mecanum,
    input: () -> Vector2d,
    rotation: () -> Double,
    fieldCentric: () -> Boolean,
    obstacleAvoidance: () -> Boolean = {false}
) : SequentialCommandGroup() {

    init {
        val rotatedInput = { input.invoke().rotated(mecanum.getPoseEstimate().heading) } // No clue why I need to do this to make it work smh
        val fieldInput = if (!fieldCentric.invoke()) rotatedInput else input

        addCommands(
            PseudoMotionProfiledDrive(
                mecanum,
                fieldInput,
                rotation
            )
        )
        addRequirements(mecanum)
    }

    override fun isFinished(): Boolean {
        return false
    }

    fun calculateForce() {

//        if (obstacleAvoidance.invoke()) {
//            var avoidanceForce = Vector2d(0.0, 0.0)
//
//            for (x in -2..2) {
//                for (y in -2..2) {
//                    val corner = Vector2d(x * RobotConfig.tileSize, y * RobotConfig.tileSize)
//                    val diff = poseEstimate.vec().minus(corner)
//                    val dist = diff.norm()
//                    val k = 0.01 // TODO put in config
//                    avoidanceForce += diff * k / (dist * dist * dist)
//                }
//            }
//
//            avoidanceForce = avoidanceForce.rotated(-poseEstimate.heading)
//            fieldFramePower += Vector2d(fieldFrameInput.x * avoidanceForce.x, fieldFrameInput.y * avoidanceForce.y)
//        }

    }

}