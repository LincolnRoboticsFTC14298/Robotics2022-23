package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

/**
 * Resets passthrough to pick up position. Finishes when angle is close enough to target.
 * @author Jared Haertel
 */
class PassthroughPickUp(private val passthrough: Passthrough) : CommandBase() {

    init {
        addRequirements(passthrough)
    }

    override fun initialize() {
        passthrough.pickUp()
    }

    override fun isFinished(): Boolean {
        return passthrough.atTargetAngle()
    }

}