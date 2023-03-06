package org.firstinspires.ftc.teamcode.teleops.testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Lift.Companion.magnetLimitName

class SensorTest() : OpMode() {

    private lateinit var limit: TouchSensor
    //private lateinit var color: NormalizedColorSensor


    override fun init() {
        limit = hardwareMap.get(TouchSensor::class.java, magnetLimitName);
        //color = hardwareMap.get(NormalizedColorSensor::class.java, "color")
    }

    override fun loop() {
        telemetry.addData("On or nah", limit.isPressed)
        telemetry.update()
    }


}