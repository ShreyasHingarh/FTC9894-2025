package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.Color;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Cannon;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;

public class Hardware {
    public final MecanumDrive drive;
    public final Cannon cannon;
    public final Camera camera;
    public final Intake intake;
    public final Sorter sorter;
    public Hardware(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        cannon = new Cannon(hardwareMap);
        intake = new Intake(hardwareMap);
        sorter = new Sorter(hardwareMap);
        camera = new Camera(hardwareMap);
    }
}
