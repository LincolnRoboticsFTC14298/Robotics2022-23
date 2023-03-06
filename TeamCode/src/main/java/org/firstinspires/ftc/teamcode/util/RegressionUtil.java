package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.Nullable;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.List;

/**
 * Various regression utilities.
 */
public class RegressionUtil {

    public static class FeedforwardResult {
        public final double kV, kA, kStatic, rSquare;

        public FeedforwardResult(double kV, double kA, double kStatic, double rSquare) {
            this.kV = kV;
            this.kA = kA;
            this.kStatic = kStatic;
            this.rSquare = rSquare;
        }
    }


    public static FeedforwardResult fitStateData(List<Double> timeSamples, List<Double> velSamples,
                                                 List<Double> accelSamples, List<Double> powerSamples,
                                                 @Nullable File file) {
        if (file != null) {
            try (PrintWriter pw = new PrintWriter(file)) {
                pw.println("time,position,power");
                for (int i = 0; i < timeSamples.size(); i++) {
                    double time = timeSamples.get(i);
                    double vel = velSamples.get(i);
                    double accel = accelSamples.get(i);
                    double power = powerSamples.get(i);
                    pw.println(time + ", " + vel + ", " + accel + ", " + power);
                }
            } catch (FileNotFoundException e) {
                // ignore
            }
        }

        double[][] x = new double[timeSamples.size()][2];
        double[] y = new double[powerSamples.size()];

        for (int i = 0; i < timeSamples.size(); i++) {
            x[i] = new double[]{velSamples.get(i), accelSamples.get(i)};
            y[i] = powerSamples.get(i);
        }

        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(y, x);

        double[] b = regression.estimateRegressionParameters();
        return new FeedforwardResult(b[1], b[2], b[0], regression.calculateAdjustedRSquared());
    }

}