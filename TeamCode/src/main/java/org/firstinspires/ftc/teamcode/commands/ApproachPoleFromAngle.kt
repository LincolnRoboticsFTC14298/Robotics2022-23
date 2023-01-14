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
        val result = null//vision.getNearesetPole()
        if (result != null) {
            addCommands(
                InstantCommand(vision::startStreamingRearCamera),
                SequentialCommandGroup(
                    // Drive normally until a cone has been detected
                    // TODO: Default command doesn't update if it is changed
                    ParallelDeadlineGroup(
                        WaitUntilCommand { false },//vision.getNeaerstPole() != null },
                        mecanum.defaultCommand
                    ),
                    ApproachAngle(
                        mecanum,
                        {0.0},//vision::getNeaerstPole,
                        speed
                    )
                ),
                InstantCommand(vision::stopStreamingRearCamera)

            )
        }
        addRequirements(mecanum, vision)

    }
}