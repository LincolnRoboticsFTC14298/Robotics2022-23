package org.firstinspires.ftc.teamcode.teleops.tuning.drive;

import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.teleops.OpModeRegister;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;
import org.firstinspires.ftc.teamcode.util.OverflowEncoder;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

final class DriveView {
    public final String type;

    public final double inPerTick;
    public final double maxVel, minAccel, maxAccel;

    public final List<LynxModule> lynxModules;

    public final List<DcMotorEx> leftMotors, rightMotors;
    public final List<DcMotorEx> motors;

    // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
    //                  (parEncs.isEmpty() && perpEncs.isEmpty())
    public final List<RawEncoder> leftEncs, rightEncs, parEncs, perpEncs;
    public final List<RawEncoder> forwardEncs;

    public final List<Encoder> forwardEncsWrapped;

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
        lynxModules = hardwareMap.getAll(LynxModule.class);

        final Localizer localizer;
        if (OpModeRegister.DRIVE_CLASS.equals(MecanumDrive.class)) {
            type = "mecanum";

            inPerTick = MecanumDrive.IN_PER_TICK;
            maxVel = MecanumDrive.MAX_WHEEL_VEL;
            minAccel = MecanumDrive.MIN_PROFILE_ACCEL;
            maxAccel = MecanumDrive.MAX_PROFILE_ACCEL;

            md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), new OdometryLocalizer(hardwareMap), new VoltageSensor(hardwareMap)); ;
            leftMotors = Arrays.asList(md.getLeftFront(), md.getLeftBack());
            rightMotors = Arrays.asList(md.getRightFront(), md.getRightBack());
            imu = md.getImu();
            voltageSensor = md.getVoltageSensor();

            localizer = md.getLocalizer();
        } else {
            throw new AssertionError();
        }

        forwardEncsWrapped = new ArrayList<>();

        if (localizer instanceof OdometryLocalizer) {
            OdometryLocalizer l3 = (OdometryLocalizer) localizer;
            parEncs = Arrays.asList(unwrap(l3.getPar0()), unwrap(l3.getPar1()));
            perpEncs = Collections.singletonList(unwrap(l3.getPerp()));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();

            forwardEncsWrapped.add(l3.getPar0());
            forwardEncsWrapped.add(l3.getPar1());
            forwardEncsWrapped.add(l3.getPerp());
        } else if (localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = Arrays.asList(unwrap(dl.getLeftFront()), unwrap(dl.getLeftRear()));
            rightEncs = Arrays.asList(unwrap(dl.getRightFront()), unwrap(dl.getRightRear()));

            forwardEncsWrapped.add(dl.getLeftFront());
            forwardEncsWrapped.add(dl.getLeftRear());
            forwardEncsWrapped.add(dl.getRightFront());
            forwardEncsWrapped.add(dl.getRightRear());
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

    public void setBulkCachingMode(LynxModule.BulkCachingMode mode) {
        for (LynxModule m : lynxModules) {
            m.setBulkCachingMode(mode);
        }
    }

    public Map<SerialNumber, Double> resetAndBulkRead(MidpointTimer t) {
        final Map<SerialNumber, Double> times = new HashMap<>();
        for (LynxModule m : lynxModules) {
            m.clearBulkCache();

            t.addSplit();
            m.getBulkData();
            times.put(m.getSerialNumber(), t.addSplit());
        }
        return times;
    }
}
