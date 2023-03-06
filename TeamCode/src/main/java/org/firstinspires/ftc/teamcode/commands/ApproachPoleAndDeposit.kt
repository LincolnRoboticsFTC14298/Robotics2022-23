package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.commands.drive.ApproachRelativePoint
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.subsystems.Vision.Companion.visionToPoleMaxDistance

/**
 * Uses localizer to align with pole most "in sight" of the robot for depositing
 * the cone while accepting driver input. It then deposits the cone and
 * resets. If there is no pole detected the command does nothing.
 * TODO: Reconsider purpose
 * @author Jared Haertel
 */
class ApproachPoleAndDeposit(
    private val mecanum: MecanumDrive,
    private val vision: Vision,
    lift: Lift,
    passthrough: Passthrough,
    claw: Claw,
    poleType: FieldConfig.PoleType,
    input: () -> Twist2d
) : SequentialCommandGroup() {

    private val nearestPole = mecanum.getClosestPoleOfType(poleType)

    init { // TODO Semi-auto based on choosing the pole + vision
        addCommands(
            ParallelCommandGroup(
                // Start the lift and extend passthrough once appropriate
                ReadyPoleDeposit(poleType, lift, passthrough),
                // Approach pole
                ApproachRelativePoint(
                    mecanum,
                    ::getVisionRelativePoint,
                    lift.getFutureRelativePosition() * passthrough.getFutureRelativePosition(),
                    input
                )
            ),
            // Once lift and passthrough are done and at the pole, deposit
            ClawDeposit(claw),
        )
        addRequirements(mecanum, lift, passthrough, claw)
    }

    /**
     * Based on the localizer estimate to supplement vision data
     * TODO TEST
     */
    private fun getVisionRelativePoint() : Vector2d {
        val local = mecanum.pose.inverse() * nearestPole.vector
        val closestVisionMatch = vision.getLandmarkInfo().minByOrNull { (it.toVector()-local).sqrNorm() }?.toVector()
        return if (closestVisionMatch != null && (closestVisionMatch-local).norm() < visionToPoleMaxDistance)
            closestVisionMatch else local
    }

}