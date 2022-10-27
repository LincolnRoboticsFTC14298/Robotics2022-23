package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CPPBridge;

/**
 * @author Tony Riggioni
 * DO NOT USE THIS ONE, IT WILL CRASH THE PHONE
 */
@TeleOp
public class BrokenTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        CPPBridge.breakShit();

        if (isStopRequested()) return;
    }
}