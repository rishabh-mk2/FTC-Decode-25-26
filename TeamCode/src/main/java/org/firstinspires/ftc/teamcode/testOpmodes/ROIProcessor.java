package org.firstinspires.ftc.teamcode.testOpmodes;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

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

    public final MatOfPoint roi5MOP = new MatOfPoint();
    public final MatOfPoint roi4MOP = new MatOfPoint();
    public final MatOfPoint roi3MOP = new MatOfPoint();
    //public final MatOfPoint roi2MOP = new MatOfPoint();
    //public final MatOfPoint roi1MOP = new MatOfPoint();

    private final MatOfPoint2f roi5Boundary2f = new MatOfPoint2f();
    private final MatOfPoint2f roi4Boundary2f = new MatOfPoint2f();
    private final MatOfPoint2f roi3Boundary2f = new MatOfPoint2f();
    //private final MatOfPoint2f roi2Boundary2f = new MatOfPoint2f();
    //private final MatOfPoint2f roi1Boundary2f = new MatOfPoint2f();

    private final List<Point> purpleSampleCenters = new ArrayList<>();
    private final List<Point> greenSampleCenters = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        Point[] leftZonePoints = {
                new Point(0, 0), new Point(400, 0),
                new Point(400, 960), new Point(0, 960)
        };
        Point[] centerZonePoints = {
                new Point(400, 0), new Point(800, 0),
                new Point(800, 960), new Point(400, 960)
        };
        Point[] rightZonePoints = {
                new Point(800, 0), new Point(1200, 0),
                new Point(1200, 960), new Point(800, 960)
        };

        roi5MOP.fromArray(leftZonePoints);
        roi4MOP.fromArray(centerZonePoints);
        roi3MOP.fromArray(rightZonePoints);
        //roi2MOP.fromArray(centerZonePoints);
        //roi1MOP.fromArray(rightZonePoints);

        roi5MOP.convertTo(roi5Boundary2f, CvType.CV_32F);
        roi4MOP.convertTo(roi4Boundary2f, CvType.CV_32F);
        roi3MOP.convertTo(roi3Boundary2f, CvType.CV_32F);
        //roi2MOP.convertTo(roi2Boundary2f, CvType.CV_32F);
        //roi1MOP.convertTo(roi1Boundary2f, CvType.CV_32F);

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

        // Erosion and Dilation starts here
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
        if (Imgproc.pointPolygonTest(roi5Boundary2f, p, false) >= 0) return 5;
        if (Imgproc.pointPolygonTest(roi4Boundary2f, p, false) >= 0) return 4;
        if (Imgproc.pointPolygonTest(roi3Boundary2f, p, false) >= 0) return 3;
        //if (Imgproc.pointPolygonTest(roi2Boundary2f, p, false) >= 0) return 2;
        //if (Imgproc.pointPolygonTest(roi1Boundary2f, p, false) >= 0) return 1;
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
        Paint leftPaint = new Paint();
        leftPaint.setColor(Color.argb(128, 255, 0, 0));
        leftPaint.setStyle(Paint.Style.FILL);

        Paint middleLeftPaint = new Paint();
        middleLeftPaint.setColor(Color.argb(128, 0, 0, 255));
        middleLeftPaint.setStyle(Paint.Style.FILL);

        Paint centerPaint = new Paint();
        centerPaint.setColor(Color.argb(128, 60, 60, 60));
        centerPaint.setStyle(Paint.Style.FILL);

        Paint middleRightPaint = new Paint();
        middleRightPaint.setColor(Color.argb(128, 0, 60, 255));
        middleRightPaint.setStyle(Paint.Style.FILL);

        Paint rightPaint = new Paint();
        rightPaint.setColor(Color.argb(128, 60, 0, 60));
        rightPaint.setStyle(Paint.Style.FILL);

        Paint purpleMarker = new Paint();
        purpleMarker.setColor(Color.MAGENTA);

        Paint greenMarker = new Paint();
        greenMarker.setColor(Color.GREEN);

        drawROI(canvas, roi5MOP, leftPaint, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi4MOP, centerPaint, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi3MOP, rightPaint, scaleBmpPxToCanvasPx);
        /*drawROI(canvas, roi5MOP, rightPaint, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi4MOP, middleRightPaint, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi3MOP, centerPaint, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi2MOP, middleLeftPaint, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi1MOP, leftPaint, scaleBmpPxToCanvasPx);*/

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