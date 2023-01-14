package org.firstinspires.ftc.teamcode.teleops.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Lift

@TeleOp
class LiftKalmanTest() : OpMode() {

    private lateinit var lift: Lift

    override fun init() {
        lift = Lift(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        val power = gamepad1.left_stick_x * 0.6
        lift.setPower(power)
        lift.testFilter()
        lift.fetchTelemetry(telemetry)
        telemetry.addData("power", power)
        telemetry.update()
    }

}