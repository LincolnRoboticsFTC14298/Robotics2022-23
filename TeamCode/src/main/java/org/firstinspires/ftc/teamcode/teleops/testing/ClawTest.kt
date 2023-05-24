package org.firstinspires.ftc.teamcode.teleops.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Claw

class ClawTest() : OpMode() {

    lateinit var claw: Claw

    override fun init() {
        claw = Claw(hardwareMap)
        claw.open()
    }

    var lastRead = false
    override fun loop() {

        lastRead = if(!lastRead && claw.isConeInside()) {
            claw.close()
            true
        } else claw.isConeInside()

        if(gamepad1.a) {
            claw.open()
        }

        claw.periodic()

        val p = TelemetryPacket()
        p.put("Is cone", claw.isConeInside())
        p.put("Last read", lastRead)
        claw.fetchTelemetry(p)
        FtcDashboard.getInstance().sendTelemetryPacket(p)
    }

}