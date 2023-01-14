package org.firstinspires.ftc.teamcode.teleops.tuning

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

@TeleOp
@Disabled
class PassthroughTuner() : OpMode() {

    lateinit var passthrough: Passthrough

    override fun init() {
        passthrough = Passthrough(hardwareMap)
    }

    override fun loop() {

        if (gamepad1.a) {
            RobotConfig.passthroughDepositAngle -= 1.0
            passthrough.deposit()
        }

        if (gamepad1.b) {
            RobotConfig.passthroughDepositAngle += 1.0
            passthrough.deposit()
        }

        if (gamepad1.x) {
            RobotConfig.passthroughPickUpAngle -= 1.0
            passthrough.pickUp()
        }

        if (gamepad1.y) {
            RobotConfig.passthroughPickUpAngle += 1.0
            passthrough.pickUp()
        }

        telemetry.addData("Deposit Angle (Degrees)", RobotConfig.passthroughDepositAngle)
        telemetry.addData("Pick Up Angle (Degrees)", RobotConfig.passthroughPickUpAngle)
        passthrough.fetchTelemetry(telemetry)
        telemetry.update()
    }
}