package org.firstinspires.ftc.teamcode.teleops.cpp_expirement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Tony Riggioni
 * DO NOT USE THIS ONE, IT WILL CRASH THE PHONE
 */
@TeleOp
@Disabled
public class BrokenTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        CPPBridge.breakShit();
    }
}