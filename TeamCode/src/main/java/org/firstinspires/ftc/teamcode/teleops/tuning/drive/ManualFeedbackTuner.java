package org.firstinspires.ftc.teamcode.teleops.tuning.drive;

import static org.firstinspires.ftc.teamcode.FieldConfig.tileSize;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;

import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.teleops.ActionOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleops.OpModeRegister;

@Config
public final class ManualFeedbackTuner extends ActionOpMode {
    public static double DISTANCE = 64;
    public static Pose2d startingPose = new Pose2d(2.0*tileSize, 2.0*tileSize, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        if (OpModeRegister.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose, new OdometryLocalizer(hardwareMap), new VoltageSensor(hardwareMap));

            waitForStart();

            while (opModeIsActive()) {
                drive.getVoltageSensor().periodic();
                drive.periodic();

                Pose2d endPose = startingPose.times(new Pose2d(DISTANCE, 0, 0));
                runBlocking(
                        drive.actionBuilder(drive.getPose())
                                .lineToX(endPose.trans.x)
                                .lineToX(startingPose.trans.x)
                                .build());
            }
        }  else {
            throw new AssertionError();
        }
    }
}
