package org.firstinspires.ftc.teamcode.teleops.testing

import android.util.Log
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

@Disabled
@TeleOp(name = "Servo + Motor Test")
class ServoMotorTest() : OpMode() {

    lateinit var servo: Servo
    lateinit var motor: DcMotor

    override fun init() {
        servo = hardwareMap.get(Servo::class.java, "servo")
        motor = hardwareMap.get(DcMotor::class.java, "motor")
        PhotonCore.enable()
    }

    val timer = ElapsedTime()
    var lastTime = timer.time()

    override fun loop() {
        if (gamepad1.a) servo.position = 0.0
        if (gamepad1.b) servo.position = 1.0
        motor.power = (-gamepad1.left_stick_y).toDouble()

        val currTime = timer.time()
        val dt = currTime-lastTime
        lastTime = currTime

        Log.i(this.javaClass.name, "Hz: " + 1.0 / dt)
        telemetry.addData("Hz", 1.0 / dt)
        telemetry.update()
    }

}