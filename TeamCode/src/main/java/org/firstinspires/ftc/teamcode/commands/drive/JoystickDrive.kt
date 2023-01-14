package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.drive.localization.MecanumMonteCarloLocalizer


/**
 * Manual joystick control of drivetrain.
 */
class JoystickDrive(
    private val mecanum: Mecanum,
    private val localizer: MecanumMonteCarloLocalizer,
    private val forward: () -> Double,
    private val strafe: () -> Double,
    private val rotation: () -> Double,
    private val fieldCentric: () -> Boolean,
    private val obstacleAvoidance: () -> Boolean = {false}
) : CommandBase() {

    init {
        addRequirements(mecanum)
    }

    override fun execute() {
        val poseEstimate: Pose2d = localizer.poseEstimate

        // Create a vector from the gamepad x/y inputs
        var input = Vector2d(forward.invoke(), strafe.invoke())

        if (fieldCentric.invoke()) {
            // Then, rotate that vector by the inverse of that heading
            input = input.rotated(-localizer.poseEstimate.heading)
        }

        var power = input

        // Calculate avoidance force using obstacles on current tile
        if (obstacleAvoidance.invoke()) {
            var avoidanceForce = Vector2d(0.0, 0.0)

            for (x in -2..2) {
                for (y in -2..2) {
                    val corner = Vector2d(x * RobotConfig.tileSize, y * RobotConfig.tileSize)
                    val diff = poseEstimate.vec().minus(corner)
                    val dist = diff.norm()
                    val k = 0.01 // TODO put in config
                    avoidanceForce += diff * k / (dist * dist * dist)
                }
            }

            avoidanceForce = avoidanceForce.rotated(-poseEstimate.heading)
            power = input + Vector2d(input.x * avoidanceForce.x, input.y * avoidanceForce.y) // TODO should input be rotated after
        }

        // Pass in the rotated input + right stick value for rotation
        // Rotation is independent of field centrism
        mecanum.setWeightedDrivePower(Pose2d(power, rotation.invoke()))
    }

}