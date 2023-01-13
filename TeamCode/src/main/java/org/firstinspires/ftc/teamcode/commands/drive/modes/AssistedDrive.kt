package org.firstinspires.ftc.teamcode.commands.drive.modes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.drive.localization.MecanumMonteCarloLocalizer

/**
 * Uses guided vector field (GVF) to avoid running into poles and
 * going to target locations, all while accepting driver input.
 * @author Jared Haertel
 */
class AssistedDrive(
    private val mecanum: Mecanum,
    private val localizer: MecanumMonteCarloLocalizer,
    private val forward: () -> Double,
    private val strafe: () -> Double,
    private val rotation: () -> Double,
    private val fieldCentric: () -> Boolean
) : CommandBase() {

    init {
        addRequirements(mecanum)
    }

    override fun execute() {
        // Create a vector from the gamepad x/y inputs
        var input = Vector2d(forward.invoke(), strafe.invoke())

        if (fieldCentric.invoke()) {
            val poseEstimate: Pose2d = localizer.poseEstimate

            // Then, rotate that vector by the inverse of that heading
            input = input.rotated(-poseEstimate.heading)
        }

        // Pass in the rotated input + right stick value for rotation
        // Rotation is independent of field centrism
        mecanum.setWeightedDrivePower(Pose2d(input, rotation.invoke()))
    }

}