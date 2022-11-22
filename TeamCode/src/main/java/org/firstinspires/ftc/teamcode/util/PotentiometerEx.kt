package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.hardware.HardwareDevice
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

interface PotentiometerEx : HardwareDevice {

    enum class Direction {
        FORWARD,
        REVERSE
    }

    /**
     * Sets the range of the potentiometer at specified angles.
     *
     * @param minAngle  The minimum value. The specified minimum is the angle the potentiometer
     *                  makes when it is at the position of 0.
     * @param maxAngle  The maximum value. The specified maximum is the angle the potentiometer
     *                  makes when it is at the position of 1.
     * @param angleUnit The unit of the range parameters
     */
    fun setRange(minAngle: Double, maxAngle: Double, angleUnit: AngleUnit = AngleUnit.DEGREES)

    /**
     * Sets the inversion factor of the servo.
     *
     * <br>By default, the inversion is false.</br>
     *
     * @param direction The desired inversion factor.
     */
    fun setInverted(direction: Direction)

    /**
     * @return true if the servo is inverted, false otherwise
     */
    fun getInverted() : Direction

    /**
     * @return The current position of the potentiometer from 0 to 1.
     */
    fun getPosition(): Double

    /**
     * @param angleUnit Angle unit for the result to be returned in
     * @return The angle the potentiometer is at.
     */
    fun getAngle(angleUnit: AngleUnit = AngleUnit.DEGREES): Double

}