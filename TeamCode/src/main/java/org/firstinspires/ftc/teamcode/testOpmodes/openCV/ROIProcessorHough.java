package org.firstinspires.ftc.teamcode.testOpmodes.openCV;

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
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class ROIProcessorHough implements VisionProcessor {

    private static final double SCALE = 0.5; // was 0.5
    private static final double INV_SCALE = 1.0 / SCALE;

    private static final Scalar PURPLE_LOW  = new Scalar(115, 61, 67);
    private static final Scalar PURPLE_HIGH = new Scalar(163, 255, 255);
    private static final Scalar GREEN_LOW   = new Scalar(35, 180, 90);
    private static final Scalar GREEN_HIGH  = new Scalar(90, 255, 255);

    private static final double H_DP = 1; // was 1.2
    private static final double H_MIN_DIST = 30;
    private static final double H_PARAM1 = 150; // was 60
    private static final double H_PARAM2 = 14;
    private static final int H_MIN_R =  5;
    private static final int H_MAX_R = 300;

    private static final double MIN_FILL_RATIO = 0.47; //was 0.42
    private static final double MIN_CIRCULARITY = 0.72; // was 0.72
    private static final double MAX_RADIUS_RATIO = 1.35;
    private static final double MIN_RADIUS_RATIO = 0.65;

    private static final Size CLOSE_SIZE  = new Size(5, 5);
    private static final Size DILATE_SIZE = new Size(2, 2); // was 3, 3
    private static final Size ERODE_SIZE  = new Size(3, 3); // was 3, 3
    private static final Size BLUR_SIZE   = new Size(5, 5);


    private Mat closeKernel, erodeKernel, dilateKernel;
    private final Mat smallFrame = new Mat();
    private final Mat hsvSmall   = new Mat();
    private final Mat colorMask  = new Mat();
    private final Mat validMask  = new Mat();
    private final Mat circles    = new Mat();


    public final MatOfPoint roi1MOP = new MatOfPoint();
    public final MatOfPoint roi2MOP = new MatOfPoint();
    public final MatOfPoint roi3MOP = new MatOfPoint();
    public final MatOfPoint roi4MOP = new MatOfPoint();
    public final MatOfPoint roi5MOP = new MatOfPoint();
    public final MatOfPoint roi6MOP = new MatOfPoint();

    private final MatOfPoint2f roi1s = new MatOfPoint2f();
    private final MatOfPoint2f roi2s = new MatOfPoint2f();
    private final MatOfPoint2f roi3s = new MatOfPoint2f();
    private final MatOfPoint2f roi4s = new MatOfPoint2f();
    private final MatOfPoint2f roi5s = new MatOfPoint2f();
    private final MatOfPoint2f roi6s = new MatOfPoint2f();

    private final List<float[]> purpleCircles = new ArrayList<>();
    private final List<float[]> greenCircles  = new ArrayList<>();

    private Paint zonePaint1, zonePaint2, zonePaint3,
            zonePaint4, zonePaint5, zonePaint6;
    private Paint purpleFill, purpleStroke, greenFill, greenStroke, dotPaint;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        Point[] zone1 = {
                new Point(183,  67), new Point(183,   0),
                new Point(217,   0), new Point(217,  67),
                new Point(  0, 313), new Point(  0, 178),
        };
        Point[] zone2 = {
                new Point(217,  67), new Point(217,   0),
                new Point(267,   0), new Point(267,  67),
                new Point( 33, 480), new Point(  0, 480),
                new Point(  0, 313),
        };
        Point[] zone3 = {
                new Point(267,  67), new Point(267,   0),
                new Point(300,   0), new Point(300,  67),
                new Point(283, 480), new Point( 33, 480),
        };
        Point[] zone4 = {
                new Point(300,  67), new Point(300,   0),
                new Point(333,   0), new Point(333,  67),
                new Point(442, 325), new Point(283, 347),
        };
        Point[] zone5 = {
                new Point(333,  67), new Point(333,   0),
                new Point(383,   0), new Point(383,  67),
                new Point(600, 311), new Point(442, 325),
        };
        Point[] zone6 = {
                new Point(383,  67), new Point(383,   0),
                new Point(640,   0), new Point(640, 298),
                new Point(600, 311),
        };

        roi1MOP.fromArray(zone1); buildScaled2f(zone1, roi1s);
        roi2MOP.fromArray(zone2); buildScaled2f(zone2, roi2s);
        roi3MOP.fromArray(zone3); buildScaled2f(zone3, roi3s);
        roi4MOP.fromArray(zone4); buildScaled2f(zone4, roi4s);
        roi5MOP.fromArray(zone5); buildScaled2f(zone5, roi5s);
        roi6MOP.fromArray(zone6); buildScaled2f(zone6, roi6s);

        closeKernel  = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, CLOSE_SIZE);
        erodeKernel  = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, ERODE_SIZE);
        dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, DILATE_SIZE);

        zonePaint1 = makeZonePaint(Color.argb(50,  79,  28,   0));
        zonePaint2 = makeZonePaint(Color.argb(50,  60,   0,  60));
        zonePaint3 = makeZonePaint(Color.argb(50,   137,  60, 255));
        zonePaint4 = makeZonePaint(Color.argb(50,  60,  60,  60));
        zonePaint5 = makeZonePaint(Color.argb(75,   0,   56, 255));
        zonePaint6 = makeZonePaint(Color.argb(75, 255,   0,   0));

        purpleFill   = makePaint(Color.argb(50, 200,   0, 200), Paint.Style.FILL);
        purpleStroke = makePaint(Color.MAGENTA, Paint.Style.STROKE);
        greenFill    = makePaint(Color.argb(50,   0, 200, 0), Paint.Style.FILL);
        greenStroke  = makePaint(Color.GREEN, Paint.Style.STROKE);
        dotPaint     = makePaint(Color.WHITE, Paint.Style.FILL);
    }

    private static void buildScaled2f(Point[] pts, MatOfPoint2f out) {
        Point[] s = new Point[pts.length];
        for (int i = 0; i < pts.length; i++)
            s[i] = new Point(pts[i].x * SCALE, pts[i].y * SCALE);
        MatOfPoint tmp = new MatOfPoint(s);
        tmp.convertTo(out, CvType.CV_32F);
        tmp.release();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.resize(frame, smallFrame, new Size(), SCALE, SCALE, Imgproc.INTER_LINEAR);
        Imgproc.cvtColor(smallFrame, hsvSmall, Imgproc.COLOR_RGB2HSV);

        synchronized (purpleCircles) {
            purpleCircles.clear();
            detectCircles(PURPLE_LOW, PURPLE_HIGH, purpleCircles);
        }
        synchronized (greenCircles) {
            greenCircles.clear();
            detectCircles(GREEN_LOW, GREEN_HIGH, greenCircles);
        }
        return null;
    }

    private void detectCircles(Scalar low, Scalar high, List<float[]> results) {
        Core.inRange(hsvSmall, low, high, colorMask);

        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, closeKernel);
        Imgproc.erode(colorMask, colorMask, erodeKernel);
        Imgproc.dilate(colorMask, colorMask, dilateKernel);

        colorMask.copyTo(validMask);

        Imgproc.GaussianBlur(colorMask, colorMask, BLUR_SIZE, 1.5);

        circles.release();
        Imgproc.HoughCircles(
                colorMask, circles,
                Imgproc.HOUGH_GRADIENT,
                H_DP, H_MIN_DIST,
                H_PARAM1, H_PARAM2,
                H_MIN_R, H_MAX_R
        );

        if (circles.empty()) return;

        int imgW = validMask.cols();
        int imgH = validMask.rows();

        for (int i = 0; i < circles.cols(); i++) {
            double[] c = circles.get(0, i);
            if (c == null) continue;

            float cx = (float) c[0];
            float cy = (float) c[1];
            float r  = (float) c[2];

            int x0 = (int) Math.max(0, Math.floor(cx - r));
            int y0 = (int) Math.max(0, Math.floor(cy - r));
            int x1 = (int) Math.min(imgW - 1, Math.ceil(cx + r));
            int y1 = (int) Math.min(imgH - 1, Math.ceil(cy + r));
            int bw = x1 - x0;
            int bh = y1 - y0;
            if (bw < 4 || bh < 4) continue;

            Mat patch = validMask.submat(new Rect(x0, y0, bw, bh));
            double coloredPx  = Core.countNonZero(patch);
            patch.release();

            double circleArea = Math.PI * r * r;
            double fillRatio  = coloredPx / circleArea;
            if (fillRatio < MIN_FILL_RATIO) continue;

            Mat patchMask = validMask.submat(new Rect(x0, y0, bw, bh));
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(patchMask.clone(), contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            hierarchy.release();
            patchMask.release();

            double bestArea  = -1;
            double bestPerim = -1;
            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area > bestArea) {
                    bestArea  = area;
                    MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
                    bestPerim = Imgproc.arcLength(cnt2f, true);
                    cnt2f.release();
                }
                cnt.release();
            }

            if (bestArea < 4 || bestPerim < 4) continue;

            double circularity = (4.0 * Math.PI * bestArea) / (bestPerim * bestPerim);
            if (circularity < MIN_CIRCULARITY) continue;

            double contourR = Math.sqrt(bestArea / Math.PI);
            double rRatio   = contourR / r;
            if (rRatio < MIN_RADIUS_RATIO || rRatio > MAX_RADIUS_RATIO) continue;

            float finalR = (float)(contourR + 1.5);

            results.add(new float[]{
                    (float)(cx  * INV_SCALE),
                    (float)(cy  * INV_SCALE),
                    (float)(finalR * INV_SCALE)
            });
        }
    }

    public List<float[]> getPurpleCircles() {
        synchronized (purpleCircles) { return new ArrayList<>(purpleCircles); }
    }

    public List<float[]> getGreenCircles() {
        synchronized (greenCircles) { return new ArrayList<>(greenCircles); }
    }

    public List<Point> getPurpleCenters() { return toCenterList(getPurpleCircles()); }
    public List<Point> getGreenCenters()  { return toCenterList(getGreenCircles());  }

    private static List<Point> toCenterList(List<float[]> list) {
        List<Point> pts = new ArrayList<>(list.size());
        for (float[] c : list) pts.add(new Point(c[0], c[1]));
        return pts;
    }

    public int getROIIndexForPoint(Point p) {
        Point ps = new Point(p.x * SCALE, p.y * SCALE);
        if (Imgproc.pointPolygonTest(roi6s, ps, false) >= 0) return 6;
        if (Imgproc.pointPolygonTest(roi5s, ps, false) >= 0) return 5;
        if (Imgproc.pointPolygonTest(roi4s, ps, false) >= 0) return 4;
        if (Imgproc.pointPolygonTest(roi3s, ps, false) >= 0) return 3;
        if (Imgproc.pointPolygonTest(roi2s, ps, false) >= 0) return 2;
        if (Imgproc.pointPolygonTest(roi1s, ps, false) >= 0) return 1;
        return 0;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {

        drawROI(canvas, roi1MOP, zonePaint1, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi2MOP, zonePaint2, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi3MOP, zonePaint3, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi4MOP, zonePaint4, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi5MOP, zonePaint5, scaleBmpPxToCanvasPx);
        drawROI(canvas, roi6MOP, zonePaint6, scaleBmpPxToCanvasPx);

        float sw   = 3 * scaleCanvasDensity;
        float dotR = 5 * scaleCanvasDensity;
        purpleStroke.setStrokeWidth(sw);
        greenStroke.setStrokeWidth(sw);

        for (float[] c : getPurpleCircles()) {
            float cx = c[0] * scaleBmpPxToCanvasPx;
            float cy = c[1] * scaleBmpPxToCanvasPx;
            float r  = c[2] * scaleBmpPxToCanvasPx;
            canvas.drawCircle(cx, cy, r,    purpleFill);
            canvas.drawCircle(cx, cy, r,    purpleStroke);
            canvas.drawCircle(cx, cy, dotR, dotPaint);
        }

        for (float[] c : getGreenCircles()) {
            float cx = c[0] * scaleBmpPxToCanvasPx;
            float cy = c[1] * scaleBmpPxToCanvasPx;
            float r  = c[2] * scaleBmpPxToCanvasPx;
            canvas.drawCircle(cx, cy, r,    greenFill);
            canvas.drawCircle(cx, cy, r,    greenStroke);
            canvas.drawCircle(cx, cy, dotR, dotPaint);
        }
    }

    private static Paint makePaint(int color, Paint.Style style) {
        Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
        p.setColor(color);
        p.setStyle(style);
        return p;
    }

    private static Paint makeZonePaint(int c) { return makePaint(c, Paint.Style.FILL); }

    private static void drawROI(Canvas canvas, MatOfPoint roi, Paint fill, float scale) {
        Point[] pts = roi.toArray();
        if (pts.length == 0) return;
        Path path = new Path();
        path.moveTo((float)(pts[0].x * scale), (float)(pts[0].y * scale));
        for (int i = 1; i < pts.length; i++)
            path.lineTo((float)(pts[i].x * scale), (float)(pts[i].y * scale));
        path.close();
        canvas.drawPath(path, fill);
    }
}