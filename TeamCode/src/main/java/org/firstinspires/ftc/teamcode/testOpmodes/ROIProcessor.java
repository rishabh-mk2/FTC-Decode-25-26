package org.firstinspires.ftc.teamcode.testOpmodes;

import android.graphics.Canvas;
import org.opencv.core.Point;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ROIProcessor implements VisionProcessor {
    private Mat mask = new Mat();
    private Mat masked = new Mat();
    private Mat ycrcb = new Mat();

    public MatOfPoint roi5MOP = new MatOfPoint();
    public MatOfPoint roi4MOP = new MatOfPoint();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        Point[] roi5 = new Point[]{
                new Point(795, 175),
                new Point(1260, 0),
                new Point(1280, 460),
                new Point(1280, 460)
        };
        Point[] roi4 = new Point[]{
                new Point(300, 175),
                new Point(500, 0),
                new Point(500, 460),
                new Point(400, 460)
        };
        mask = new Mat(height, width, CvType.CV_8UC1);
        roi5MOP = new MatOfPoint(roi5);
        roi4MOP = new MatOfPoint(roi4);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (masked.empty()){
            masked = new Mat(frame.size(), frame.type());
        }
        if (ycrcb.empty()){
            ycrcb = new Mat(frame.size(), frame.type());
        }
        Imgproc.cvtColor(frame, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        mask.setTo(Scalar.all(0));
        Imgproc.fillConvexPoly(mask, roi5MOP, Scalar.all(255));
        Imgproc.fillConvexPoly(mask, roi4MOP, Scalar.all(255));
        masked.setTo(Scalar.all(0));
        frame.copyTo(masked, mask);

        return null;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext
    ) {
        // ---------- Paints ----------

        // 75% black background
        android.graphics.Paint bgPaint = new android.graphics.Paint();
        bgPaint.setColor(android.graphics.Color.argb(191, 0, 0, 0)); // 75% opacity
        bgPaint.setStyle(android.graphics.Paint.Style.FILL);

        // ROI 5: dark purple (75%)
        android.graphics.Paint roi5Paint = new android.graphics.Paint();
        roi5Paint.setColor(android.graphics.Color.argb(191, 128, 0, 200));
        roi5Paint.setStyle(android.graphics.Paint.Style.FILL);

        // ROI 4: lighter purple (75%)
        android.graphics.Paint roi4Paint = new android.graphics.Paint();
        roi4Paint.setColor(android.graphics.Color.argb(191, 180, 80, 255));
        roi4Paint.setStyle(android.graphics.Paint.Style.FILL);

        // White outline
        android.graphics.Paint outlinePaint = new android.graphics.Paint();
        outlinePaint.setColor(android.graphics.Color.WHITE);
        outlinePaint.setStyle(android.graphics.Paint.Style.STROKE);
        outlinePaint.setStrokeWidth(4 * scaleCanvasDensity);

        // ---------- Background ----------
        canvas.drawRect(0, 0, onscreenWidth, onscreenHeight, bgPaint);

        // ---------- ROI 5 ----------
        org.opencv.core.Point[] pts5 = roi5MOP.toArray();
        android.graphics.Path path5 = new android.graphics.Path();
        path5.moveTo((float) pts5[0].x * scaleBmpPxToCanvasPx,
                (float) pts5[0].y * scaleBmpPxToCanvasPx);
        for (int i = 1; i < pts5.length; i++) {
            path5.lineTo((float) pts5[i].x * scaleBmpPxToCanvasPx,
                    (float) pts5[i].y * scaleBmpPxToCanvasPx);
        }
        path5.close();

        canvas.drawPath(path5, roi5Paint);
        canvas.drawPath(path5, outlinePaint);

        // ---------- ROI 4 ----------
        org.opencv.core.Point[] pts4 = roi4MOP.toArray();
        android.graphics.Path path4 = new android.graphics.Path();
        path4.moveTo((float) pts4[0].x * scaleBmpPxToCanvasPx,
                (float) pts4[0].y * scaleBmpPxToCanvasPx);
        for (int i = 1; i < pts4.length; i++) {
            path4.lineTo((float) pts4[i].x * scaleBmpPxToCanvasPx,
                    (float) pts4[i].y * scaleBmpPxToCanvasPx);
        }
        path4.close();

        canvas.drawPath(path4, roi4Paint);
        canvas.drawPath(path4, outlinePaint);
    }

}
