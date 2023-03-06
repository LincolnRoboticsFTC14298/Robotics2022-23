package org.firstinspires.ftc.teamcode.teleops.tuning.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.teleops.TuningOpModes;
import org.firstinspires.ftc.teamcode.teleops.ActionOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public final class SplineTest extends ActionOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), new OdometryLocalizer(hardwareMap), new VoltageSensor(hardwareMap));

            waitForStart();

            runBlocking(
                drive.actionBuilder(drive.getPose())
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(60, 0), Math.PI)
                        .build());
        }else {
            throw new AssertionError();
        }
    }
}
