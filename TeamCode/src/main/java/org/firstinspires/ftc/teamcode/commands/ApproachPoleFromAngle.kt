package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.commands.drive.ApproachAngle
import org.firstinspires.ftc.teamcode.subsystems.*

class ApproachPoleFromAngle(
    mecanum: Mecanum,
    vision: Vision,
    speed: () -> Double
) : SequentialCommandGroup() {

    init {
        addCommands(
            InstantCommand(vision::startStreamingFrontCamera),
            SequentialCommandGroup(
                // Drive normally until a cone has been detected
                ParallelDeadlineGroup(
                    WaitUntilCommand { vision.getClosestPoleAngle() != null },
                    mecanum.defaultCommand
                ),
                ApproachAngle(
                    mecanum,
                    vision::getClosestPoleAngle,
                    speed
                )
            ),
            InstantCommand(vision::stopStreamingFrontCamera)

        )
        addRequirements(mecanum, vision)
    }
}