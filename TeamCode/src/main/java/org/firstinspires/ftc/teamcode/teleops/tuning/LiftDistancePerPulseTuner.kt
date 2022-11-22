package org.firstinspires.ftc.teamcode.teleops.tuning

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Lift

@TeleOp
class LiftDistancePerPulseTuner() : OpMode() {

    private val lift: Lift = Lift(hardwareMap, "liftLeft", "liftRight")

    override fun init() {

    }

    override fun loop() {
        telemetry.addData("Position", lift.getHeight())
        telemetry.update()
    }

}