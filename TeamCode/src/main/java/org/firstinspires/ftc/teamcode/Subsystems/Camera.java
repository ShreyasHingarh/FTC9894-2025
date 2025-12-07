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

//Port to roadrunner with 2 wheel odometry
// Auto sorting
// Auto lineup to goal + speed c/ launch + parking
// Turn on intake

//20 blue goal
//21: Green Purple Purple
//22: Purple Green Purple
//23: Purple Purple Green
//24 red goal

public class Camera {
    private AprilTagProcessor aprilTag;
    public BallColor[] Order;
    public Camera(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
    }
    private void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
    }

    public double[] telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                return new double[] {detection.id, detection.ftcPose.x, detection.ftcPose.y};
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        return new double[]{-1. -1.-1};
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
