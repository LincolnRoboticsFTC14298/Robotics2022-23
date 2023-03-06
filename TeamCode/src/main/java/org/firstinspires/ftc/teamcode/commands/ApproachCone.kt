package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.Twist2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.commands.drive.ApproachRelativePoint
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Drives until it detects a cone using vision. It then precisely aligns
 * and drives to pick up the cone with input from driver. Uses vision
 * because environment is much more dynamic than a pole.
 *
 * TODO: use GVF
 */
class ApproachCone(
    mecanum: MecanumDrive,
    vision: Vision,
    lift: Lift,
    passthrough: Passthrough,
    claw: Claw,
    input: () -> Twist2d
) : SequentialCommandGroup() {

    init {
        addCommands(
            InstantCommand(vision::startStreamingRearCamera),
            ParallelDeadlineGroup(
                // Waiting for intake to get a cone
                WaitUntilCommand { claw.isConeInside() },
                // Driving
                ApproachRelativePoint(
                    mecanum,
                    { vision.getClosestCone(mecanum.pose)?.toVector() },
                    lift.getFutureRelativePosition() * passthrough.getFutureRelativePosition(),
                    { Twist2d(-input.invoke().transVel, input.invoke().rotVel) }
                )
            ),
            InstantCommand(vision::startStreamingRearCamera)
        )
        addRequirements(mecanum, vision, lift, passthrough, claw)
    }

}