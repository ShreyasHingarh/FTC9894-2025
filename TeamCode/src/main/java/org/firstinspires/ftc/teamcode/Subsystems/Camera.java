package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Enums.BallColor;
import org.firstinspires.ftc.teamcode.Enums.Color;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import com.acmerobotics.roadrunner.Action;

public class Camera {
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public BallColor[] Order;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    public Camera(HardwareMap hardwareMap) {
        initVision(hardwareMap);
    }
    public OpenCVBallDetection ballProcessor;

    private void initVision(HardwareMap hardwareMap) {
        ballProcessor = new OpenCVBallDetection();
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

         visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)      // Processor 1
                .addProcessor(ballProcessor) // Processor 2
                .build();
    }

    @SuppressLint("DefaultLocale")
    public double[] telemetryAprilTag(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            // Only use tags that don't have Obelisk in them
            if (!detection.metadata.name.contains("Obelisk") && (detection.id == 24 || detection.id == 20)) {
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                return new double[] {detection.id, detection.robotPose.getPosition().x,detection.robotPose.getPosition().y,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES), detection.ftcPose.range, detection.ftcPose.yaw};
            }

//            if (detection.metadata != null) {
//
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
            return new double[] {detection.id};
        }
        return new double[]{-1.0};
    }

    public Action setOrderFromTag(Telemetry telemetry){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double[] data = telemetryAprilTag(telemetry);
                if(data[0] == 21){
                    Order = new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple};
                    return true;
                } else if(data[0] == 22){
                    Order = new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
                    return true;
                } else if(data[0] == 23){
                    Order = new BallColor[]{BallColor.Purple, BallColor.Purple, BallColor.Green};
                    return true;
                }
                return false;
            }
        };
    }

    public boolean PickOrder (Gamepad gamepad){
        if(gamepad.x){
            Order = new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple};
            return true;
        } else if(gamepad.a){
            Order = new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
            return true;
        } else if(gamepad.b){
            Order = new BallColor[]{BallColor.Purple, BallColor.Purple, BallColor.Green};
            return true;
        }
        return false;
    }
    public double getCannonPower(Telemetry telemetry){
        double[] data = telemetryAprilTag(telemetry);
        if(data[0] != 24 || data[0] != 20) return -0.8;
        if(data[4] > 100){
            return -1;
        } else if(data[4] > 70){
            return -0.87;
        }
        return -0.8;
    }
}