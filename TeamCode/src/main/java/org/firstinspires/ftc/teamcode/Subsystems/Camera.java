package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enums.BallColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import com.acmerobotics.roadrunner.Action;

public class Camera {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public BallColor[] Order;

    public Camera(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
    }

    private void initAprilTag(HardwareMap hardwareMap) {
        // NOTE: CenterStage Library only contains IDs 1-10.
        // If you are using IDs 21-24, they will show as "Unknown" unless you build a custom library.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public double[] telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addData("Id", detection.id);
            // Returns the first valid tag found
            return new double[] {detection.id};

        }
        return new double[]{-1.0};
    }

    public boolean setOrderFromTag(Telemetry telemetry){
        double[] a = telemetryAprilTag(telemetry);
        if(a[0] == 21){
            Order = new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple};
            return true;
        } else if(a[0] == 22){
            Order = new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
            return true;
        } else if(a[0] == 23){
            Order = new BallColor[]{BallColor.Purple, BallColor.Purple, BallColor.Green};
            return true;
        }
        return false;
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
}