package org.firstinspires.ftc.teamcode.teleops.tuning

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Lift

@TeleOp
@Disabled
class LiftDistancePerPulseTuner() : OpMode() {

    private val lift: Lift = Lift(hardwareMap)

    override fun init() {

    }

    override fun loop() {
        telemetry.addData("Position", lift.getCurrentHeight() - RobotConfig.liftHeightOffset)
        telemetry.update()
    }

}