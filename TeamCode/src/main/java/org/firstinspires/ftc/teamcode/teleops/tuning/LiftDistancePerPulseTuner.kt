package org.firstinspires.ftc.teamcode.teleops.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Lift
import java.lang.Math.toRadians
import kotlin.math.sin

@TeleOp
class LiftDistancePerPulseTuner() : OpMode() {

    private lateinit var lift: Lift

    override fun init() {
        lift = Lift(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        val power = -gamepad1.left_stick_y * 0.6

        lift.checkLimit = true
        lift.checkEncoder()
        lift.setPower(power)
        lift.updateFilter(null)
        telemetry.addData("power", power)
        lift.fetchTelemetry(telemetry)
        telemetry.update()
    }

}