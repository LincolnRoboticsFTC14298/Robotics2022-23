package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Lift
import kotlin.math.abs

/**
 * Responsible for commanding the lift to go to a specified height using a [PIDFController].
 * @author Jared Haertel
 */
class LiftHeight(private val height: Double, private val lift: Lift) : CommandBase() {

    /**
     * Specified heights in centimeters.
     */
    enum class Height(val height: Double) {
        DEFAULT(0.0),
        LOW_POLE(0.0),
        MIDDLE_POLE(0.0),
        HIGH_POLE(0.0)
    }

    constructor(height: Height, lift: Lift) : this(height.height, lift)

    init {
        addRequirements(lift)
    }

    private val coeffs = PIDCoefficients(0.0, 0.0, 0.0)
    private val controller = PIDFController(coeffs,  kStatic = 0.0)

    // TODO: Find max values
    private val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
        MotionState(0.0, 0.0, 0.0),
        MotionState(height, 0.0, 0.0),
        25.0,                             // cm / s
        40.0,                           // cm / s2
        100.0                            // cm / s3
    )

    private val timer = ElapsedTime()

    private val errorTolerance = 0.5 // cm

    override fun initialize() {
        timer.reset()

        controller.reset() // Probably not necessary
        controller.targetPosition = height
    }

    override fun execute() {
        val state = motionProfile[timer.seconds()] // TODO: is it seconds?

        controller.apply {
            targetPosition = state.x
            targetVelocity = state.v
            targetAcceleration = state.a
        }

        val power = controller.update(lift.getHeight(), lift.getVelocity())
        lift.setPower(power)
    }

    override fun end(interrupted: Boolean) {
        lift.setPower(0.0) // Acts like brake
    }

    override fun isFinished(): Boolean {
        return abs(controller.lastError) < errorTolerance
    }

    fun timeFromFinished(): Double {
        return motionProfile.duration() - timer.seconds()
    }

}