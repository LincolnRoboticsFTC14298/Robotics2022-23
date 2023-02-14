package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughDepositAngle
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughPickUpAngle
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Responsible for lowering [Lift] and setting the [Passthrough]
 * in the right position for pick up. The [Claw] should remain off.
 * @author Jared Haertel
 */
class ReadyPickUp(lift: Lift, claw: Claw, passthrough: Passthrough) : SequentialCommandGroup() {

    init{
        addCommands(
            InstantCommand(lift::retract, lift),
            InstantCommand(passthrough::pickUp, passthrough),
            WaitUntilCommand { passthrough.getAngleEstimate() <= 90.0 }, // TODO It can technically be > 90
            InstantCommand(claw::open, claw)
        )
        addRequirements(lift, claw, passthrough)
    }

}