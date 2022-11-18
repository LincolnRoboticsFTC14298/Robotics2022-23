package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

interface PotentiometerEx : HardwareDevice {

    /**
     * Sets the range of the potentiometer at specified angles.
     *
     * @param min       The minimum value. The specified minimum is the angle the potentiometer
     *                  makes when it is at the position of 0.
     * @param max       The maximum value. The specified maximum is the angle the potentiometer
     *                  makes when it is at the position of 1.
     * @param angleUnit The unit of the range parameters
     */
    fun setRange(minAngle: Double, maxAngle: Double, angleUnit: AngleUnit = AngleUnit.DEGREES)

    /**
     * Sets the inversion factor of the servo.
     *
     * <p>By default, the inversion is false.</p>
     *
     * @param isInverted the desired inversion factor
     */
    fun setInverted(isInverted: Boolean)

    /**
     * @return true if the servo is inverted, false otherwise
     */
    fun getInverted() : Boolean

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