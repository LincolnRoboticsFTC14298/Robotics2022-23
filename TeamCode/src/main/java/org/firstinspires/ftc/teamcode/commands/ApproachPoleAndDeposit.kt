package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.visionToPoleMaxDistance
import org.firstinspires.ftc.teamcode.commands.drive.ApproachRelativePoint
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Uses localizer to align with pole most "in sight" of the robot for depositing
 * the cone while accepting driver input. It then deposits the cone and
 * resets. If there is no pole detected the command does nothing.
 * TODO: Reconsider purpose
 * @author Jared Haertel
 */
class ApproachPoleAndDeposit(
    private val mecanum: Mecanum,
    private val vision: Vision,
    lift: Lift,
    passthrough: Passthrough,
    claw: Claw,
    poleType: RobotConfig.PoleType,
    input: () -> Vector2d
) : SequentialCommandGroup() {

    private val nearestPole = mecanum.getClosestPoleOfType(poleType)

    init { // TODO Semi-auto based on choosing the pole + vision
        addCommands(
            InstantCommand(vision::startStreamingFrontCamera),
            ParallelCommandGroup(
                // Start the lift and extend passthrough once appropriate
                ReadyPoleDeposit(poleType, lift, passthrough),
                // Approach pole
                ApproachRelativePoint(
                    mecanum,
                    ::getVisionRelativePoint,
                    lift.getFutureRelativePosition() + passthrough.getFutureRelativePosition(),
                    input
                )
            ),
            // Once lift and passthrough are done and at the pole, deposit
            ClawDeposit(claw),
            InstantCommand(vision::stopStreamingFrontCamera)
        )
        addRequirements(mecanum, lift, passthrough, claw)
    }

    /**
     * Based on the localizer estimate, it obtains the closest pole data
     * TODO TEST
     */
    private fun getVisionRelativePoint() : Vector2d {
        val diff = nearestPole.vector - mecanum.getPoseEstimate().vec()
        val closestVisionMatch = vision.getLandmarkInfo().minBy { it.toVector().distTo(diff) }.toVector()
        return if (closestVisionMatch.distTo(diff) < visionToPoleMaxDistance) closestVisionMatch else diff
    }

}