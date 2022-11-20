package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Responsible for lowering [Lift] and setting the [Passthrough] in the right position for pick up.
 * The [Intake] should remain off.
 * TODO: May want to check logic to minimize lift vibration/strain
 * @author Jared Haertel
 */
class ReadyPickUp(lift: Lift, passthrough: Passthrough) : ParallelCommandGroup() {

    init{
        addCommands(
            LiftHeight(LiftHeight.Height.DEFAULT, lift),
            PassthroughPickUp(passthrough)
        )
        addRequirements(lift, passthrough)
    }

}