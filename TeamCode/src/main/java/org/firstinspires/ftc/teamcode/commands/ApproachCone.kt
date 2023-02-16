package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.*
import org.firstinspires.ftc.teamcode.commands.drive.ApproachRelativePoint
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Passthrough
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.Vision

/**
 * Drives until it detects a cone using vision. It then precisely aligns
 * and drives to pick up the cone with input from driver. Uses vision
 * because environment is much more dynamic than a pole.
 */
class ApproachCone(
    mecanum: Mecanum,
    vision: Vision,
    lift: Lift,
    passthrough: Passthrough,
    claw: Claw,
    input: () -> Vector2d
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
                    vision::getClosestConePosition,
                    lift.getFutureRelativePosition() + passthrough.getFutureRelativePosition(),
                    input
                )
            ),
            InstantCommand(vision::startStreamingRearCamera)
        )
        addRequirements(mecanum, vision, lift, passthrough, claw)
    }

}