package org.firstinspires.ftc.teamcode.vision.testing;

import static org.opencv.core.Core.bitwise_or;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.RETR_EXTERNAL;
import static org.opencv.imgproc.Imgproc.arcLength;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.boxPoints;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.convexHull;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.minAreaRect;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.rectangle;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestPipeline4 extends OpenCvPipeline {

    Telemetry telemetry;

    public TestPipeline4(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat realInput = input.clone();

        //colorspace
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);

        //filter by color
        Mat inputRed = new Mat();
        Mat inputBlue = new Mat();
        Mat inputPole = new Mat();

        inRange(input, new Scalar(0, 150, 100), new Scalar(255, 200, 175), inputRed);
        inRange(input, new Scalar(0, 130, 30), new Scalar(255, 180, 120), inputBlue);
        inRange(input, new Scalar(0, 110, 150), new Scalar(255, 150, 200), inputPole);
        //Core.bitwise_or(inputRed, inputBlue, input);
        input=inputPole;

        //morphology
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_CLOSE, Mat.ones(5,3, CvType.CV_32F));
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_OPEN, Mat.ones(3,1, CvType.CV_32F));

        //find and draw contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(input,contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        drawContours(realInput, contours,-1, new Scalar(0,255,0), 1);

        //draw raw mask (optional)
        Mat output = Mat.zeros(input.size(), CvType.CV_8U);
        drawContours(output, contours, -1, new Scalar(255), -1);

        //scorers
        List<MatOfPoint> hullList = new ArrayList<>();


        for (int i=0; i<contours.size(); i++) {
            MatOfPoint contour = contours.get(i);

            Rect boundingBox = boundingRect(contour);
            double height = boundingBox.height;
            double width = boundingBox.width;

            double contourArea = contourArea(contour);
            double contourPerimeter = arcLength(new MatOfPoint2f(contour.toArray()), true);

            Moments moments = moments(contour);
            Point centroid = new Point();
            centroid.x = moments.get_m10() / moments.get_m00();
            centroid.y = moments.get_m01() / moments.get_m00();

            MatOfInt hull = new MatOfInt();
            convexHull(contour, hull);

            Point[] contourArray = contour.toArray();
            Point[] hullPoints = new Point[hull.rows()];
            List<Integer> hullContourIdxList = hull.toList();
            for (int j = 0; j < hullContourIdxList.size(); j++) {
                hullPoints[j] = contourArray[hullContourIdxList.get(j)];
            }

            MatOfPoint convexHull = new MatOfPoint(hullPoints);
            hullList.add(convexHull);


            double hullPerimeter = arcLength(new MatOfPoint2f(convexHull.toArray()),true);
            double hullArea = contourArea(convexHull);

            /// New variable
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect contourMinAreaRect = minAreaRect(contour2f);
            Mat contourBoxPoints = new Mat();
            boxPoints(contourMinAreaRect, contourBoxPoints);

            double solidity = contourArea / hullArea;
            double aspectRatio = height/width;
            double extent = contourArea / (boundingBox.width * boundingBox.height);
            double convexity = hullPerimeter / contourPerimeter;
            double distanceByWidth = Math.min(contourMinAreaRect.size.width, contourMinAreaRect.size.height); //TODO double check, finish formula


            String testScorer = "solidity";
            telemetry.addData("centroid #" + (i+1), centroid);
            telemetry.addData(testScorer + " #" + (i+1), solidity);

            //rectangle(realInput, boundingBox, new Scalar(255,0,0));
        }
        //drawContours(realInput, hullList,-1, new Scalar(255,0,0), 2, Imgproc.LINE_8);

        telemetry.update();
        return realInput;
    }
}
















