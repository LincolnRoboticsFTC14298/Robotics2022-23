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

        telemetry.addData("Deposit Angle (Degrees)", RobotConfig.passthroughDepositAngle)
        telemetry.addData("Pick Up Angle (Degrees)", RobotConfig.passthroughPickUpAngle)
        passthrough.fetchTelemetry(telemetry)
        telemetry.update()
    }
}