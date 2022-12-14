package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Intake

/**
 * Reverses intake to spit out cone and stops after a predetermined amount of time.
 * @author Jared Haertel
 */
class IntakeDeposit(private val intake: Intake) : CommandBase() {

    private val timer: ElapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

    init {
        addRequirements(intake)
    }

    override fun initialize() {
        timer.reset()
        intake.spitOut()
    }

    override fun isFinished(): Boolean {
        return timer.milliseconds() >= RobotConfig.intakeTimeToSpitOut && !intake.isConeInside()
    }

    override fun end(interrupted: Boolean) {
        intake.off()
    }

}