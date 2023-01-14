package org.firstinspires.ftc.teamcode.commands

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
 * TODO: Test!! What happens when the cone tips? When it is lifted? When it moves? What if its too close? How will it react?
 */
class ApproachCone(
    mecanum: Mecanum,
    vision: Vision,
    lift: Lift,
    passthrough: Passthrough,
    claw: Claw,
    speed: () -> Double
) : SequentialCommandGroup() {

    init {
        addCommands(
            InstantCommand(vision::startStreamingFrontCamera),
            ParallelDeadlineGroup(
                // Waiting for intake to get a cone
                WaitUntilCommand { claw.isConeInside() },
                // Driving
                SequentialCommandGroup(
                    // Drive normally until a cone has been detected
                    // TODO: Default command doesn't update if it is changed
                    ParallelDeadlineGroup(
                        WaitUntilCommand { vision.getSingleConeRelativePosition() != null },
                        mecanum.defaultCommand
                    ),
                    // Switch to auto approach the cone once a cone has been detected
                    ApproachRelativePoint(
                        mecanum,
                        vision::getSingleConeRelativePosition,
                        lift.getRelativePosition() + passthrough.getRelativePosition(),
                        speed
                    )
                )
            ),
            InstantCommand(vision::stopStreamingFrontCamera)
        )
        addRequirements(mecanum, vision, claw)
    }

}