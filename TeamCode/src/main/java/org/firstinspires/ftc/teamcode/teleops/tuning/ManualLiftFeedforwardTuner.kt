package org.firstinspires.ftc.teamcode.teleops.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor

class ManualLiftFeedforwardTuner : OpMode() {

    lateinit var lift: Lift

    override fun init() {
        lift = Lift(hardwareMap, VoltageSensor(hardwareMap))
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    var up = false
    override fun loop() {
        if (lift.timeFromTarget() < 0 ) {
            if(!up) {
                lift.setpoint = 30.0
                up = true
            }
            else {
                lift.setpoint = 5.0
                up = false
            }
        }

        lift.periodic()
        lift.fetchTelemetry(telemetry)
    }
}