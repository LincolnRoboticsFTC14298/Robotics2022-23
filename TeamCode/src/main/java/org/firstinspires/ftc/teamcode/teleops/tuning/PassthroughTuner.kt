package org.firstinspires.ftc.teamcode.teleops.tuning

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

@TeleOp
class PassthroughTuner() : OpMode() {

    lateinit var passthrough: Passthrough

    override fun init() {
        passthrough = Passthrough(hardwareMap)
    }

    override fun loop() {

        if (gamepad1.dpad_down) {
            passthrough.pickUp()
        }

        if (gamepad1.dpad_right) {
            passthrough.junctionDeposit()
        }

        if (gamepad1.dpad_up) {
            passthrough.deposit()
        }

        if (gamepad1.a) {
            passthrough.setPosition(0.0)
        }

        telemetry.addData("Time to deposit", passthrough.timeToTarget(RobotConfig.passthroughDepositAngle))
        telemetry.addData("Time to junction deposit", passthrough.timeToTarget(RobotConfig.passthroughJunctionAngle))
        telemetry.addData("Time to pick up", passthrough.timeToTarget(RobotConfig.passthroughPickUpAngle))
        passthrough.fetchTelemetry(telemetry)
        telemetry.update()
    }
}