package org.firstinspires.ftc.teamcode.teleops.tuning.drive;

import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.teleops.OpModeManager;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.OverflowEncoder;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

final class DriveView {
    public final String type;

    public final double inPerTick;
    public final double maxVel, minAccel, maxAccel;

    public final List<DcMotorEx> leftMotors, rightMotors;
    public final List<DcMotorEx> motors;

    // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
    //                  (parEncs.isEmpty() && perpEncs.isEmpty())
    public final List<RawEncoder> leftEncs, rightEncs, parEncs, perpEncs;
    public final List<RawEncoder> forwardEncs;

    public final IMU imu;

    public final VoltageSensor voltageSensor;

    private final MecanumDrive md;

    private static RawEncoder unwrap(Encoder e) {
        if (e instanceof OverflowEncoder) {
            return ((OverflowEncoder) e).encoder;
        } else {
            return (RawEncoder) e;
        }
    }

    public DriveView(HardwareMap hardwareMap) {
        final Localizer localizer;
        if (OpModeManager.DRIVE_CLASS.equals(MecanumDrive.class)) {
            type = "mecanum";

            inPerTick = MecanumDrive.IN_PER_TICK;
            maxVel = MecanumDrive.MAX_WHEEL_VEL;
            minAccel = MecanumDrive.MIN_PROFILE_ACCEL;
            maxAccel = MecanumDrive.MAX_PROFILE_ACCEL;

            md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), new OdometryLocalizer(hardwareMap), new VoltageSensor(hardwareMap));

            leftMotors = Arrays.asList(md.getLeftFront(), md.getLeftBack());
            rightMotors = Arrays.asList(md.getRightFront(), md.getRightBack());
            imu = md.getImu();
            voltageSensor = md.getVoltageSensor();

            localizer = md.getLocalizer();
        } else {
            throw new AssertionError();
        }

        if (localizer instanceof OdometryLocalizer) {
            OdometryLocalizer l3 = (OdometryLocalizer) localizer;
            parEncs = Arrays.asList(unwrap(l3.getPar0()), unwrap(l3.getPar1()));
            perpEncs = Collections.singletonList(unwrap(l3.getPerp()));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();
        } else if (localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = Arrays.asList(unwrap(dl.getLeftFront()), unwrap(dl.getLeftRear()));
            rightEncs = Arrays.asList(unwrap(dl.getRightFront()), unwrap(dl.getRightRear()));
        } else {
            throw new AssertionError();
        }

        motors = new ArrayList<>();
        motors.addAll(leftMotors);
        motors.addAll(rightMotors);

        forwardEncs = new ArrayList<>();
        forwardEncs.addAll(leftEncs);
        forwardEncs.addAll(rightEncs);
        forwardEncs.addAll(parEncs);

        List<RawEncoder> allEncs = new ArrayList<>();
        allEncs.addAll(forwardEncs);
        allEncs.addAll(perpEncs);

        DcMotorController c1 = allEncs.get(0).getController();
        for (Encoder e : allEncs) {
            DcMotorController c2 = e.getController();
            if (c1 != c2) {
                throw new IllegalArgumentException("all encoders must be attached to the same hub");
            }
        }
    }

    public MotorFeedforward feedforward() {
        if (md != null) {
            return new MotorFeedforward(MecanumDrive.kS, MecanumDrive.kV, MecanumDrive.kA);
        }

        throw new AssertionError();
    }

    public void setDrivePowers(Twist2d powers) {
        if (md != null) {
            md.setDrivePowers(powers);
            return;
        }

        throw new AssertionError();
    }
}
