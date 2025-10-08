package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.ActionRunner;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.Node;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

/// Position 0,0,0 is refering to the center of the robot
public class Pathing {

    public Node Path;
    private Node current;
    public static Node createPath(Node node) {
        while (node.Previous != null) {
            node = node.Previous;
        }
        return node;
    }
    public Pathing(Node path){
        Path = path;
        current = path;
    }
    public boolean RunPath(MecanumDrive drive){

        // Preprocessing if at a new point when creating the points (will occur before waitForStart();
        // calculate the velocity, acceleration and position curves of the path that are needed
        // I have current (x, y, theta) and the next (x, y, theta)
        if(current != null){
            // when running this now, I have the target, I get the actual from the encoders and the gyro
            // and then pid the difference
            // move from current to current.next
            // pass that in to the setDrivePowers function()
            PoseVelocity2d poseVelocity2d = new PoseVelocity2d(new Vector2d(1,2), 0);
            drive.setDrivePowers(poseVelocity2d);
            return false;
        }
        return true;
    }
}