package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.commands.drive.ApproachAngle
import org.firstinspires.ftc.teamcode.subsystems.*

class ApproachPoleFromAngle(
    mecanum: Mecanum,
    vision: Vision,
    input: () -> Vector2d
) : SequentialCommandGroup() {

    init {
        addCommands(
            //InstantCommand(vision::startStreamingFrontCamera),
            // Drive normally until a cone has been detected
            ApproachAngle(
                mecanum,
                vision::getClosestPoleAngle,
                input
            ),
            //InstantCommand(vision::stopStreamingFrontCamera)
        )
        addRequirements(mecanum, vision)
    }
}