package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.Potentiometer
import kotlin.math.abs


/**
 * Passthrough subsystem consist of a servo that rotates
 * the mechanism containing the intake/claw from pick up to drop off.
 * @param hw HardwareMap
 * @param servoName Passthrough servo's name
 */
class Passthrough(hwMap: HardwareMap, servoName: String, potentiometerName: String) : SubsystemBase() {

    /**
     * TODO: Set min and max degrees for servo and potentiometer.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware">FTCLib Docs: Hardware</a>
     */
    private val servo: ServoEx = SimpleServo(hwMap, servoName, 0.0, 1.0)
    private val potentiometer: Potentiometer = Potentiometer(hwMap, potentiometerName, 0.0, 0.0)

    private val min: Double = 0.0;
    private val max: Double = 1.0;


    /**
     * Rotate servo to drop off position.
     */
    fun drop() {
        servo.position = max
    }

    /**
     * Rotate servo to pick up cone.
     */
    fun pickUp() {
        servo.position = min
    }

    fun atTargetAngle(errorTolerance: Double = 0.05): Boolean {
        return abs(servo.angle - potentiometer.getAngle()) < errorTolerance
    }

}