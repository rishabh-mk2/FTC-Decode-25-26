package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class ROIProcessorHough implements VisionProcessor {
    private static final Scalar PURPLE_LOW_HSV  = new Scalar(130, 50, 50);
    private static final Scalar PURPLE_HIGH_HSV = new Scalar(170, 255, 255);
    private static final Scalar GREEN_LOW_HSV   = new Scalar(40, 50, 50);
    private static final Scalar GREEN_HIGH_HSV  = new Scalar(90, 255, 255);

    private final Mat grayMat = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat circlesMat = new Mat();

    private final List<Point> purpleCenters = Collections.synchronizedList(new ArrayList<>());
    private final List<Point> greenCenters = Collections.synchronizedList(new ArrayList<>());

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
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(frame, grayMat, Imgproc.COLOR_RGB2GRAY);
        // TODO: if too much noise, increase new Size param
        Imgproc.GaussianBlur(grayMat, grayMat, new Size(9,9), 2, 2);

        purpleCenters.clear();
        greenCenters.clear();
        //TODO: tune DP (resolution from my understanding) so it works for smaller circles (edge cases)
        // TODO: Tune min dist between centers to doesn't miss nor detects false positives
        // TODO:Tune param2; if getting false positives, increase; else decrease param2
        // TODO: Tune minRadius and maxRadius based on real data
        Imgproc.HoughCircles(
                grayMat,
                circlesMat,
                Imgproc.HOUGH_GRADIENT,
                1.5,
                20,
                150,
                100,
                50,
                400
        );

        for (int i = 0; i < circlesMat.cols(); i++ ){
            double[] data = circlesMat.get(0, i);
            double x = data[0];
            double y = data[1];
            Point center = new Point(x, y);
            //TODO: if code ever crashes, use the commented code below (AI GIVEN)
           /*if (x < 0 || x >= hsvMat.cols() || y < 0 || y >= hsvMat.rows()) {
               continue;
           }*/

            double[] pixelColor = hsvMat.get((int) y, (int) x);

            if (isColorInBounds(pixelColor, PURPLE_LOW_HSV, PURPLE_HIGH_HSV)){
                purpleCenters.add(center);
            } else if (isColorInBounds(pixelColor, GREEN_LOW_HSV, GREEN_HIGH_HSV)){
                greenCenters.add(center);
            }
        }
        return null;
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
        synchronized (purpleCenters) { return new ArrayList<>(purpleCenters); }
    }

    public List<Point> getGreenCenters() {
        synchronized (greenCenters) { return new ArrayList<>(greenCenters); }
    }
    private boolean isColorInBounds(double[] hsv, Scalar low, Scalar high) {
        return hsv[0] >= low.val[0] && hsv[0] <= high.val[0] &&
                hsv[1] >= low.val[1] && hsv[1] <= high.val[1] &&
                hsv[2] >= low.val[2] && hsv[2] <= high.val[2];
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
       /*drawROI(canvas, roi3MOP, centerPaint, scaleBmpPxToCanvasPx);
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
