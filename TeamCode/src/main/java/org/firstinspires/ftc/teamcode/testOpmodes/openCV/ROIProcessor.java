package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
@Disabled
public class ROIProcessor implements VisionProcessor {
    private static final Scalar PURPLE_LOW_HSV  = new Scalar(130, 50, 50);
    private static final Scalar PURPLE_HIGH_HSV = new Scalar(170, 255, 255);
    private static final Scalar GREEN_LOW_HSV   = new Scalar(40, 50, 50);
    private static final Scalar GREEN_HIGH_HSV  = new Scalar(90, 255, 255);

    private static final double MIN_DETECTION_AREA = 3000.0;
    private static final double CIRCULARITY_THRESHOLD = 0.75;


    private final Mat hsvFrame = new Mat();
    private final Mat colorThresholdMask = new Mat();
    private final Mat contourHierarchy = new Mat();
    private Mat morphKernel;

    public final MatOfPoint roi6MOP = new MatOfPoint();
    public final MatOfPoint roi5MOP = new MatOfPoint();
    public final MatOfPoint roi4MOP = new MatOfPoint();
    public final MatOfPoint roi3MOP = new MatOfPoint();
    public final MatOfPoint roi2MOP = new MatOfPoint();
    public final MatOfPoint roi1MOP = new MatOfPoint();

    private final MatOfPoint2f roi6Boundary2f = new MatOfPoint2f();
    private final MatOfPoint2f roi5Boundary2f = new MatOfPoint2f();
    private final MatOfPoint2f roi4Boundary2f = new MatOfPoint2f();
private final MatOfPoint2f roi3Boundary2f = new MatOfPoint2f();
private final MatOfPoint2f roi2Boundary2f = new MatOfPoint2f();
    private final MatOfPoint2f roi1Boundary2f = new MatOfPoint2f();



private final List<Point> purpleSampleCenters = new ArrayList<>();
    private final List<Point> greenSampleCenters = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        Point[] zone1 = {
                new Point(550, 150),
                new Point(550, 0),
                new Point(650, 0),
                new Point(650, 150),
                new Point(0, 700),
                new Point(0, 400),
        };
        Point[] zone2 = {
                new Point(650, 150),
                new Point(650, 0),
                new Point(800, 0),
                new Point(800, 150),
                new Point(100, 1080),
                new Point(0, 1080),
                new Point(0, 700),
        };
        Point[] zone3 = {
                new Point(800, 150),
                new Point(800, 0),
                new Point(900, 0),
                new Point(900, 150),
                new Point(850, 1080),
                new Point(100, 1080),
        };
        Point[] zone4 = {
                new Point(900, 150),
                new Point(900, 0),
                new Point(1000, 0),
                new Point(1000, 150),
                new Point(1325, 730),
                new Point(850, 780),
        };
        Point[] zone5 = {
                new Point(1000, 150),
                new Point(1000, 0),
                new Point(1150, 0),
                new Point(1150, 150),
                new Point(1800, 700),
                new Point(1325, 730)
        };
        Point[] zone6 = {
                new Point(1150, 150),
                new Point(1150, 0),
                new Point(1920, 0),
                new Point(1920, 670),
                new Point(1800, 700)
        };

        roi6MOP.fromArray(zone6);
        roi5MOP.fromArray(zone5);
        roi4MOP.fromArray(zone4);
        roi3MOP.fromArray(zone3);
        roi2MOP.fromArray(zone2);
        roi1MOP.fromArray(zone1);


        roi6MOP.convertTo(roi6Boundary2f, CvType.CV_32F);
        roi5MOP.convertTo(roi5Boundary2f, CvType.CV_32F);
        roi4MOP.convertTo(roi4Boundary2f, CvType.CV_32F);
        roi3MOP.convertTo(roi3Boundary2f, CvType.CV_32F);
        roi2MOP.convertTo(roi2Boundary2f, CvType.CV_32F);
        roi1MOP.convertTo(roi1Boundary2f, CvType.CV_32F);


        morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        synchronized (purpleSampleCenters) {
            purpleSampleCenters.clear();
            detectAndSplit(PURPLE_LOW_HSV, PURPLE_HIGH_HSV, purpleSampleCenters);
        }

        synchronized (greenSampleCenters) {
            greenSampleCenters.clear();
            detectAndSplit(GREEN_LOW_HSV, GREEN_HIGH_HSV, greenSampleCenters);
        }

        return null;
    }

    private void detectAndSplit(Scalar low, Scalar high, List<Point> results) {
        Core.inRange(hsvFrame, low, high, colorThresholdMask);
        Imgproc.erode(colorThresholdMask, colorThresholdMask, morphKernel, new Point(-1, -1), 0);
        Imgproc.dilate(colorThresholdMask, colorThresholdMask, morphKernel, new Point(-1, -1), 0);

        List<MatOfPoint> rawContours = new ArrayList<>();
        Imgproc.findContours(colorThresholdMask, rawContours, contourHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : rawContours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea < MIN_DETECTION_AREA) continue;

            MatOfPoint2f contourPoly = new MatOfPoint2f(contour.toArray());
            double contourPerimeter = Imgproc.arcLength(contourPoly, true);
            double circularityMetric = (4 * Math.PI * contourArea) / (Math.pow(contourPerimeter, 2));

            RotatedRect minAreaRect = Imgproc.minAreaRect(contourPoly);
            double rectWidth = minAreaRect.size.width;
            double rectHeight = minAreaRect.size.height;
            double longSide = Math.max(rectWidth, rectHeight);
            double shortSide = Math.min(rectWidth, rectHeight);

            if (circularityMetric >= CIRCULARITY_THRESHOLD) {
                results.add(minAreaRect.center);
            } else {
                int subObjectCount = (int) Math.round(longSide / shortSide);

                if (subObjectCount <= 1) {
                    results.add(minAreaRect.center);
                } else {
                    double orientationAngle = minAreaRect.angle;
                    if (rectWidth < rectHeight) orientationAngle += 90;
                    double angleRad = Math.toRadians(orientationAngle);

                    double unitVectorX = Math.cos(angleRad);
                    double unitVectorY = Math.sin(angleRad);

                    for (int i = 0; i < subObjectCount; i++) {
                        double centerShift = (i - (subObjectCount - 1) / 2.0) * (longSide / subObjectCount);
                        results.add(new Point(minAreaRect.center.x + centerShift * unitVectorX, minAreaRect.center.y + centerShift * unitVectorY));
                    }
                }
            }
        }
    }

    public int getROIIndexForPoint(Point p) {
        if (Imgproc.pointPolygonTest(roi6Boundary2f, p, false) >= 0) return 6;
        if (Imgproc.pointPolygonTest(roi5Boundary2f, p, false) >= 0) return 5;
        if (Imgproc.pointPolygonTest(roi4Boundary2f, p, false) >= 0) return 4;
        if (Imgproc.pointPolygonTest(roi3Boundary2f, p, false) >= 0) return 3;
        if (Imgproc.pointPolygonTest(roi2Boundary2f, p, false) >= 0) return 2;
        if (Imgproc.pointPolygonTest(roi1Boundary2f, p, false) >= 0) return 1;
        return 0;
    }

    public List<Point> getPurpleCenters() {
        synchronized (purpleSampleCenters) { return new ArrayList<>(purpleSampleCenters); }
    }

    public List<Point> getGreenCenters() {
        synchronized (greenSampleCenters) { return new ArrayList<>(greenSampleCenters); }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint zone6 = new Paint();
        zone6.setColor(Color.argb(128, 255, 0, 0));
        zone6.setStyle(Paint.Style.FILL);

        Paint zone5 = new Paint();
        zone5.setColor(Color.argb(128, 0, 0, 255));
        zone5.setStyle(Paint.Style.FILL);

        Paint zone4 = new Paint();
        zone4.setColor(Color.argb(128, 60, 60, 60));
        zone4.setStyle(Paint.Style.FILL);

        Paint zone3 = new Paint();
        zone3.setColor(Color.argb(128, 0, 60, 255));
        zone3.setStyle(Paint.Style.FILL);

        Paint zone2 = new Paint();
        zone2.setColor(Color.argb(128, 60, 0, 60));
        zone2.setStyle(Paint.Style.FILL);

        Paint zone1 = new Paint();
        zone1.setColor(Color.argb(128, 79, 28, 0));
        zone1.setStyle(Paint.Style.FILL);

        Paint purpleMarker = new Paint();
        purpleMarker.setColor(Color.MAGENTA);

        Paint greenMarker = new Paint();
        greenMarker.setColor(Color.GREEN);

        drawROI(canvas, roi6MOP, zone6, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi5MOP, zone5, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi4MOP, zone4, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi3MOP, zone3, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi2MOP, zone2, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi1MOP, zone1, scaleBmpPxToCanvasPx);

        float radius = 10 * scaleCanvasDensity;
        for (Point p : getPurpleCenters()) {
            canvas.drawCircle((float)p.x * scaleBmpPxToCanvasPx, (float)p.y * scaleBmpPxToCanvasPx, radius, purpleMarker);
        }
        for (Point p : getGreenCenters()) {
            canvas.drawCircle((float)p.x * scaleBmpPxToCanvasPx, (float)p.y * scaleBmpPxToCanvasPx, radius, greenMarker);
        }
    }

    private void drawROI(Canvas canvas, MatOfPoint roi, Paint fill, float scale) {
        Point[] pts = roi.toArray();
        if (pts.length == 0) return;
        Path drawPath = new Path();
        drawPath.moveTo((float) pts[0].x * scale, (float) pts[0].y * scale);
        for (int i = 1; i < pts.length; i++) drawPath.lineTo((float) pts[i].x * scale, (float) pts[i].y * scale);
        drawPath.close();
        canvas.drawPath(drawPath, fill);
    }
}