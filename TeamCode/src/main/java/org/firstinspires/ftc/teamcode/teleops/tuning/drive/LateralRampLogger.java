package org.firstinspires.ftc.teamcode.teleops.tuning.drive;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class LateralRampLogger extends LinearOpMode {
    private static double power(double seconds) {
        return Math.min(0.25 * seconds, 0.9);
    }

    private static class Signal {
        public final List<Double> times = new ArrayList<>();
        public final List<Double> values = new ArrayList<>();
    }

    private static void recordEncoderData(Encoder e, Map<SerialNumber, Double> ts, Signal ps, Signal vs) {
        SerialNumber sn = ((LynxDcMotorController) e.getController()).getSerialNumber();
        Encoder.PositionVelocityPair p = e.getPositionAndVelocity();

        ps.times.add(ts.get(sn));
        ps.values.add((double) p.position);

        vs.times.add(ts.get(sn));
        vs.values.add((double) p.velocity);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(hardwareMap);
        view.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        class Data {
            public final String type = view.type;

            public final List<Signal> frblPowers = new ArrayList<>();

            public final List<Signal> flbrPowers = new ArrayList<>();

            public final Signal voltages = new Signal();

            public final List<Signal> perpEncPositions = new ArrayList<>();

            public final List<Signal> perpEncVels = new ArrayList<>();
        }

        Data data = new Data();
        for (DcMotorEx m : view.flbr) {
            data.flbrPowers.add(new Signal());
        }
        for (DcMotorEx m : view.frbl) {
            data.frblPowers.add(new Signal());
        }
        for (Encoder e : view.perpEncs) {
            data.perpEncPositions.add(new Signal());
            data.perpEncVels.add(new Signal());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        while (opModeIsActive()) {
            view.voltageSensor.periodic();

            for (int i = 0; i < view.flbr.size(); i++) {
                double power = power(t.seconds());
                view.flbr.get(i).setPower(power);

                Signal s = data.flbrPowers.get(i);
                s.times.add(t.addSplit());
                s.values.add(power);
            }

            for (int i = 0; i < view.frbl.size(); i++) {
                double power = -power(t.seconds());
                view.frbl.get(i).setPower(power);

                Signal s = data.frblPowers.get(i);
                s.times.add(t.addSplit());
                s.values.add(power);
            }

            data.voltages.values.add(view.voltageSensor.getVoltage());
            data.voltages.times.add(t.addSplit());

            Map<SerialNumber, Double> encTimes = view.resetAndBulkRead(t);

            for (int i = 0; i < view.perpEncs.size(); i++) {
                recordEncoderData(
                        view.perpEncs.get(i),
                        encTimes,
                        data.perpEncPositions.get(i),
                        data.perpEncVels.get(i)
                );
            }
        }

        for (DcMotorEx m : view.motors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.LATERAL_RAMP, data);
    }
}
