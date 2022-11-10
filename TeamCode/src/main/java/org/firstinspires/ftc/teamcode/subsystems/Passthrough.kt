package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap


/**
 * Passthrough subsystem consist of a servo that rotates
 * the mechanism containing the intake/claw from pick up to drop off.
 * @param hwMap HardwareMap
 * @param servoName Passthrough's servo's name
 */
class Passthrough(hwMap: HardwareMap, servoName: String) : SubsystemBase()
{

    /**
     * TODO: Set min and max degrees.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware">FTCLib Docs: Hardware</a>
     */
    private val servo: ServoEx = SimpleServo(hwMap, servoName, 0.0, 1.0)

    private val MIN = 0.0;
    private val MAX = 1.0;

    /**
     * Rotate servo to drop off position.
     */
    fun drop()
    {
        servo.position = MAX
    }

    /**
     * Rotate servo to pick up cone.
     */
    fun pickUp()
    {
        servo.position = MIN
    }
}