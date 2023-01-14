package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughDepositAngle
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Responsible for lowering [Lift] and setting the [Passthrough]
 * in the right position for pick up. The [Claw] should remain off.
 * @author Jared Haertel
 */
class ReadyPickUp(lift: Lift, claw: Claw, passthrough: Passthrough, depositAngle: Double = passthroughDepositAngle) : SequentialCommandGroup() {

    init{
        addCommands(
            InstantCommand(lift::retract, lift),
            InstantCommand(claw::open, claw),
            InstantCommand({ passthrough.setpoint = depositAngle }, passthrough)
        )
        addRequirements(lift, claw, passthrough)
    }

}