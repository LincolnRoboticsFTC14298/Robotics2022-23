package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import java.lang.Math.toDegrees
import java.lang.Math.toRadians

/**
 * "Wrapper class" for potentiometer
 * @param hw        HardwareMap.
 * @param minAngle  The minimum value. The specified minimum is the angle the potentiometer
 *                  makes when it is at the position of 0.
 * @param maxAngle  The maximum value. The specified maximum is the angle the potentiometer
 *                  makes when it is at the position of 1.
 * @param angleUnit The unit of the range parameters
 */
class Potentiometer(
    hw: HardwareMap,
    name: String,
    private var minAngle: Double,
    private var maxAngle: Double,
    angleUnit: AngleUnit = AngleUnit.DEGREES)
    : PotentiometerEx {

    init {
        minAngle = toRadians(minAngle, angleUnit)
        maxAngle = toRadians(maxAngle, angleUnit)
    }

    private val potentiometer: AnalogInput = hw.get(AnalogInput::class.java, name)

    private var isInverted: Boolean = false

    override fun setRange(min: Double, max: Double, angleUnit: AngleUnit) {
        this.minAngle = toRadians(min, angleUnit)
        this.maxAngle = toRadians(max, angleUnit)
    }

    override fun setInverted(isInverted: Boolean) {
        this.isInverted = isInverted
    }

    override fun getInverted(): Boolean {
        return isInverted
    }

    override fun getPosition(): Double {
        val position = potentiometer.voltage / potentiometer.maxVoltage
        return if (isInverted) 1 - position else position
    }

    override fun getAngle(angleUnit: AngleUnit): Double {
        val angle = getPosition() * getAngleRange(angleUnit) + minAngle
        return fromRadians(angle, angleUnit)
    }

    fun getAngleRange(angleUnit: AngleUnit = AngleUnit.DEGREES): Double {
        return fromRadians(maxAngle - minAngle, angleUnit)
    }

    override fun disable() {
        potentiometer.close()
    }

    override fun getDeviceType(): String {
        return ("Potentiometer " + potentiometer.deviceName + " from " + potentiometer.manufacturer
                + "; " + potentiometer.connectionInfo)
    }

    private fun toRadians(angle: Double, angleUnit: AngleUnit): Double {
        return if (angleUnit == AngleUnit.DEGREES) toRadians(angle) else angle
    }

    private fun fromRadians(angle: Double, angleUnit: AngleUnit): Double {
        return if (angleUnit == AngleUnit.DEGREES) toDegrees(angle) else angle
    }

}