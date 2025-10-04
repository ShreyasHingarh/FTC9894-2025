package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.Color;
import org.firstinspires.ftc.teamcode.RoadRunnerThings.MecanumDrive;

public class Hardware {
    public final MecanumDrive drive;
    public Hardware(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }
}
