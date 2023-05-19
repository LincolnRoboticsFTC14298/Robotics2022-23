package org.firstinspires.ftc.teamcode.teleops.tuning.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.teleops.OpModeRegister;

public final class LateralPushTest extends LinearOpMode {
    private static double lateralSum(MecanumDrive.DriveLocalizer dl) {
        return 0.25 * (
                -dl.getLeftFront().getPositionAndVelocity().position
                + dl.getLeftRear().getPositionAndVelocity().position
                - dl.getRightRear().getPositionAndVelocity().position
                + dl.getRightFront().getPositionAndVelocity().position);
    }

    @Override
    public void runOpMode() throws InterruptedException {
            if (!OpModeRegister.DRIVE_CLASS.equals(MecanumDrive.class)) {
            throw new RuntimeException(getClass().getSimpleName() + " is for mecanum drives only.");
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), new OdometryLocalizer(hardwareMap), new VoltageSensor(hardwareMap));

        MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) drive.getLocalizer();

        drive.getLeftFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.getLeftBack().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.getRightBack().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.getRightFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        double initLateralSum = lateralSum(dl);
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", lateralSum(dl) - initLateralSum);
            telemetry.update();
        }
    }
}
