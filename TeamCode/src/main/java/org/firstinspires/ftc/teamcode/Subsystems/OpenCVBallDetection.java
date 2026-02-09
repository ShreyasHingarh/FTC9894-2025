package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Canvas;
import android.graphics.Paint;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

@Config
public class OpenCVBallDetection implements VisionProcessor {
    private Mat purpleInRange = new Mat();
    private Mat greenInRange = new Mat();
    private Mat purpleBlurred = new Mat();
    private Mat greenBlurred = new Mat();
    private Mat hierarchy = new Mat();

    private static final Scalar lowerPurple = new Scalar(111, 10, 180); // RGB Order
    private static final Scalar upperPurple = new Scalar(196, 108, 255);
    private static final Scalar lowerGreen = new Scalar(0, 131, 127); // RGB Order
    private static final Scalar upperGreen = new Scalar(112, 255, 255);
    private static final Size purpleBlur = new Size(15,15);
    private static final Size greenBlur = new Size(11,11);
    private static final double MIN_AREA = 500.0;

    public int PurpleCount = 0;
    public int GreenCount= 0;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // --- Masks ---
        Core.inRange(frame, lowerPurple, upperPurple, purpleInRange);
        Imgproc.GaussianBlur(purpleInRange, purpleBlurred, purpleBlur, 0);
        Imgproc.threshold(purpleBlurred, purpleBlurred, 127, 255, Imgproc.THRESH_BINARY);

        Core.inRange(frame, lowerGreen, upperGreen, greenInRange);
        Imgproc.blur(greenInRange, greenBlurred, greenBlur);
        Imgproc.threshold(greenBlurred, greenBlurred, 127, 255, Imgproc.THRESH_BINARY);


        List<RotatedRect> detectedRects = new ArrayList<>();
        //For green
        List<MatOfPoint> contours = new ArrayList<>();
        contours.clear();
        Imgproc.findContours(greenBlurred, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        GreenCount = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_AREA) {
                GreenCount++;
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);
                detectedRects.add(rect);
                contour2f.release();
            }
        }
        for (MatOfPoint c : contours) c.release();

        //For Purple
        contours.clear();
        Imgproc.findContours(purpleBlurred, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        PurpleCount = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_AREA) {
                PurpleCount++;
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);
                detectedRects.add(rect);
                contour2f.release();
            }
        }
        for (MatOfPoint c : contours) c.release();
        return detectedRects;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (userContext == null) return;

        Paint rectPaint = new Paint();
        rectPaint.setColor(android.graphics.Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(4.0f);

        List<RotatedRect> rects = (List<RotatedRect>) userContext;
        for (RotatedRect rect : rects) {
            float centerX = (float) rect.center.x * scaleBmpPxToCanvasPx;
            float centerY = (float) rect.center.y * scaleBmpPxToCanvasPx;
            canvas.drawCircle(centerX, centerY, 10, rectPaint);
        }
    }
}