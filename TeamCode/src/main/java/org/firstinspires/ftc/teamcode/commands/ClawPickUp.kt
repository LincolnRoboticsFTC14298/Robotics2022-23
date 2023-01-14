package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Claw

/**
 * Closes claw to pick up cone.
 * @author Jared Haertel
 */
class ClawPickUp(private val claw: Claw) : CommandBase() {

    init {
        addRequirements(claw)
    }

    override fun initialize() {
        claw.close()
    }

    override fun isFinished(): Boolean {
        return claw.atTarget()
    }

}