package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

/**
 * Sets passthrough to deposit position. Finishes when angle is close enough to target.
 * @author Jared Haertel
 */
class PassthroughDeposit(private val passthrough: Passthrough) : CommandBase() {

    init {
        addRequirements(passthrough)
    }

    override fun initialize() {
        passthrough.deposit()
    }

    override fun isFinished(): Boolean {
        return passthrough.atTargetAngle()
    }

}