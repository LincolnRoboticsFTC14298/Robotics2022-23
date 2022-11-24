package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughDepositAngle
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughMaxDegree
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughMinDegree
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughRetractedAngle
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughServoLeftName
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughServoRightName
import org.firstinspires.ftc.teamcode.RobotConfig.potentiometerName
import org.firstinspires.ftc.teamcode.RobotConfig.potentiometerOffset
import org.firstinspires.ftc.teamcode.util.Potentiometer
import kotlin.math.abs


/**
 * Passthrough subsystem consist of a servo that rotates
 * the mechanism containing the intake/claw from pick up to drop off.
 * The angle is that made with the heading vector (i.e. w/ the front of the intake side)
 * @param hwMap             HardwareMap
 */
class Passthrough(hwMap: HardwareMap) : SubsystemBase() {

    /**
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware">FTCLib Docs: Hardware</a>
     */
    private val servoLeft: ServoEx = SimpleServo(
        hwMap,
        passthroughServoLeftName,
        passthroughMinDegree,
        passthroughMaxDegree
    )
    private val servoRight: ServoEx = SimpleServo(
        hwMap,
        passthroughServoRightName,
        passthroughMinDegree,
        passthroughMaxDegree
    )

    private val potentiometer: Potentiometer = Potentiometer(
        hwMap,
        potentiometerName,
        0.0 + potentiometerOffset,
        270.0 + potentiometerOffset
    )

    var depositOffset = 0.0

    // TODO: check direction of servo

    /**
     * Rotate servo to drop off position.
     */
    fun deposit() {
        servoLeft.turnToAngle(passthroughDepositAngle + depositOffset)
        servoRight.turnToAngle(passthroughDepositAngle + depositOffset)
    }

    /**
     * Rotate servo to pick up cone.
     */
    fun pickUp() {
        servoLeft.turnToAngle(passthroughRetractedAngle)
        servoRight.turnToAngle(passthroughRetractedAngle)
    }

    fun atTargetAngle(errorTolerance: Double = 0.05): Boolean {
        return abs(servoLeft.angle - potentiometer.getAngle()) < errorTolerance
    }

}