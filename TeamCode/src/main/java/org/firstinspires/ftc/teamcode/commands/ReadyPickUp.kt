package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Responsible for lowering [Lift] and setting the [Passthrough]
 * in the right position for pick up. The [Intake] should remain off.
 * @author Jared Haertel
 */
class ReadyPickUp(lift: Lift, passthrough: Passthrough) : SequentialCommandGroup() {

    init{
        addCommands(
            InstantCommand(lift::retract, lift),
            PassthroughPickUp(passthrough)
        )
        addRequirements(lift, passthrough)
    }

}