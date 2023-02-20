package org.firstinspires.ftc.teamcode.teleops.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Claw


@TeleOp
@Disabled
class ClawTuner() : OpMode() {

    lateinit var claw: Claw

    override fun init() {
        claw = Claw(hardwareMap)
        claw.close()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        if (gamepad1.a) {
            claw.close()
        }

        if (gamepad1.b) {
            claw.open()
        }

        if (gamepad1.x) {
            claw.partiallyOpen()
        }

        claw.periodic()
        claw.fetchTelemetry(telemetry)
        telemetry.update()
    }

}