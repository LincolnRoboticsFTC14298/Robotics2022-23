package org.firstinspires.ftc.teamcode.teleops.testing

import android.util.Log
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.*
import kotlin.math.sin

@TeleOp(name = "PhotonTest")
@Disabled
class PhotonTest : OpMode() {

    lateinit var motor1: DcMotor
    lateinit var motor2: DcMotor
    lateinit var motor3: DcMotor
    lateinit var motor4: DcMotor
    private val motorList = mutableListOf<DcMotor>()

    lateinit var servo1: Servo
    lateinit var servo2: Servo
    lateinit var servo3: Servo
    lateinit var servo4: Servo
    private val servoList = mutableListOf<Servo>()

    override fun init() {
        motor1 = hardwareMap.get(DcMotor::class.java, "motor1")
        motor2 = hardwareMap.get(DcMotor::class.java, "motor2")
        motor3 = hardwareMap.get(DcMotor::class.java, "motor3")
        motor4 = hardwareMap.get(DcMotor::class.java, "motor4")
        motorList.addAll(listOf(motor1, motor2, motor3, motor4))

        servo1 = hardwareMap.get(Servo::class.java, "servo1")
        servo2 = hardwareMap.get(Servo::class.java, "servo2")
        servo3 = hardwareMap.get(Servo::class.java, "servo3")
        servo4 = hardwareMap.get(Servo::class.java, "servo4")
        servoList.addAll(listOf(servo1, servo2, servo3, servo4))

        PhotonCore.enable()
    }

    val timer = ElapsedTime()
    var lastTime = timer.seconds()
    var enabled = true

    val pastDTs = LinkedList<Double>()
    val windowSize = 15
    override fun loop() {

        for (motor in motorList) {
            motor.power = sin(timer.time())
        }

        for (servo in servoList) {
            servo.position = 0.5*(sin(timer.time())+1.0)
        }

        if (gamepad1.a) {
            PhotonCore.enable()
            enabled = true
            pastDTs.clear()
        }
        if (gamepad1.b) {
            PhotonCore.disable()
            enabled = false
            pastDTs.clear()
        }

        val currTime = timer.time()
        val dt = currTime-lastTime
        lastTime = currTime
        pastDTs.add(dt)
        if (pastDTs.size > windowSize) {
            pastDTs.pop()
        }
        val hz = 1.0 / pastDTs.average()

        Log.i("Loop time", "Hz: $hz")
        Log.i("PhotonCore", enabled.toString())
        telemetry.addData("Current Hz", 1.0 / dt)
        telemetry.addData("Averaged Hz", hz)
        telemetry.addData("PhotonCore On", enabled)
        telemetry.update()
    }
}