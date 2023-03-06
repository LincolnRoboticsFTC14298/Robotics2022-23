package org.firstinspires.ftc.teamcode.teleops.testing

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Lift

class LiftManualTest : OpMode() {

    private lateinit var motorLeft: DcMotor
    private lateinit var motorRight: DcMotor
    override fun init() {
        motorLeft = hardwareMap.dcMotor.get(Lift.leftLiftName)
        motorRight = hardwareMap.dcMotor.get(Lift.rightLiftName)
    }

    override fun loop() {
        val power = -gamepad1.left_stick_y * 0.4
        motorLeft.power = power
        motorRight.power = power
    }
}