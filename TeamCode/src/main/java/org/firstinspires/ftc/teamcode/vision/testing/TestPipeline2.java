package org.firstinspires.ftc.teamcode.vision.testing;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.LINE_8;
import static org.opencv.imgproc.Imgproc.RETR_EXTERNAL;
import static org.opencv.imgproc.Imgproc.arcLength;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.morphologyEx;
import static org.opencv.imgproc.Imgproc.rectangle;

import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestPipeline2 extends OpenCvPipeline {

    Telemetry telemetry;

    public TestPipeline2(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat realInput = input.clone();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
        inRange(input, new Scalar(0, 163, 50), new Scalar(255, 255, 255), input);
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_CLOSE, Mat.ones(5,5, CvType.CV_32F));
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_OPEN, Mat.ones(3,3, CvType.CV_32F));

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(input,contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        drawContours(realInput, contours,-1, new Scalar(0,255,0), 2, Imgproc.LINE_8);
        for (int i=0; i<contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect boundingBox = boundingRect(contour);
            double contourArea = contourArea(contour);
            double contourPerimeter = arcLength(new MatOfPoint2f(contour.toArray()), true);
            Moments moments = moments(contour);
            Point centroid = new Point();
            centroid.x = moments.get_m10() / moments.get_m00();
            centroid.y = moments.get_m01() / moments.get_m00();
            double width = (contourPerimeter/4)-(sqrt(((contourPerimeter*contourPerimeter)/4)-4*contourArea)/2);
            double height = (contourPerimeter/2)-width;
            telemetry.addData("centroid #"+i, centroid);
            telemetry.addData("extent #"+i, (double) contourArea/(boundingBox.width*boundingBox.height));
            telemetry.addData("score", 100*Math.pow(2.0, -0.5*(Math.pow((((double) contourArea/(boundingBox.width*boundingBox.height))-0.65)/0.08, 2.0))));
            double discriminant = (((contourPerimeter * contourPerimeter) / 4) - 4 * contourArea)/2;
            Rect equivalentRectangle = new Rect((int) (centroid.x-(width/2)),(int) (centroid.y-(height/2)),(int) width,(int) height);
            rectangle(realInput, boundingBox, new Scalar(255,0,0));
        }

        telemetry.update();
        return realInput;
    }
}
















