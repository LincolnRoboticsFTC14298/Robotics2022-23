package org.firstinspires.ftc.teamcode.teleops.tuning.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;

import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.teleops.ActionOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleops.OpModeRegister;


public final class ManualFeedbackTuner extends ActionOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        if (OpModeRegister.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), new OdometryLocalizer(hardwareMap), new VoltageSensor(hardwareMap));

            waitForStart();

            while (opModeIsActive()) {
                runBlocking(
                        drive.actionBuilder(drive.getPose())
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
        }  else {
            throw new AssertionError();
        }
    }
}
