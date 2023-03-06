package org.firstinspires.ftc.teamcode.subsystems

import android.graphics.Color
import android.util.Log
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.constantProfile
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range.clip
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.clawClosedPosition
import org.firstinspires.ftc.teamcode.RobotConfig.clawOpenedPosition
import org.firstinspires.ftc.teamcode.RobotConfig.clawPartiallyOpenedPosition
import org.firstinspires.ftc.teamcode.RobotConfig.clawServoName
import org.firstinspires.ftc.teamcode.RobotConfig.colorGain
import org.firstinspires.ftc.teamcode.RobotConfig.colorSensorName
import org.firstinspires.ftc.teamcode.RobotConfig.valueThreshold

/**
 * Claw subsystem consists of a servo, potentiometer and a color sensor.
 * @param hwMap HardwareMap.
 */
class Claw(hwMap: HardwareMap, startingPosition: Double = clawClosedPosition) : SubsystemBase() {

    private val servo = SimpleServo(hwMap, clawServoName, 0.0, 360.0)
    private var colorSensor = hwMap.get(NormalizedColorSensor::class.java, colorSensorName)

    private var timer = ElapsedTime()
    private lateinit var motionProfile: TimeProfile

    private var setpoint: Double = 0.0
        set(position) {
            field = clip(position, 0.0, 1.0)
            timer.reset()
            motionProfile = TimeProfile(constantProfile(position - getPositionEstimate(), 0.0, RobotConfig.clawMaxVel, -RobotConfig.clawMaxAccel, RobotConfig.clawMaxAccel).baseProfile)
            Log.i("Claw desired position", setpoint.toString())
        }

    init {
        colorSensor.gain = colorGain.toFloat()
        setpoint = startingPosition
        servo.position = startingPosition
    }

    override fun periodic() {
        Log.v("Claw estimated angle", getPositionEstimate().toString())
        servo.position = motionProfile[timer.seconds()].value()
    }

    /**
     * Open to pick up the cone
     */
    fun open() {
        setpoint = clawOpenedPosition
    }

    fun partiallyOpen() {
        setpoint = clawPartiallyOpenedPosition
    }

    /**
     * Turn servo on at a constant speed to spit out cone.
     */
    fun close() {
        setpoint = clawClosedPosition
    }

    /**
     * Confirms if claw is at target.
     */
    fun atTarget() : Boolean {
        return timer.seconds() > motionProfile.duration
    }

    private fun getPositionEstimate() : Double {
        return servo.position
    }

    /**
     * Check if there is a cone based on color sensor.
     * @return [Boolean] of state.
     */
    private val hsvValues = FloatArray(3)
    fun isConeInside(): Boolean {
        val colors = colorSensor.normalizedColors
        Color.colorToHSV(colors.toColor(), hsvValues)
        Log.i("Color Sensor Values", hsvValues.toString())
        return hsvValues[2] >= valueThreshold
    }

    /**
     * For debugging/tuning purposes
     */
    fun fetchTelemetry(telemetry: Telemetry) {
        telemetry.addData("Position Estimate", getPositionEstimate())
        telemetry.addData("Desired position", setpoint)
    }

    fun drawClaw(canvas: Canvas, clawOffset: Vector2d, pose: Pose2d) {
        canvas.setStroke("#8CA231")
        val (x, y) = pose.trans.plus(
            pose.rot.inverse().times(
                clawOffset
            )
        )
        canvas.fillCircle(x, y, RobotConfig.coneDiameter / 2.0 * 1.2)
    }

}