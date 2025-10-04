package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.Color;

public class MainActions {

    private ElapsedTime timer = new ElapsedTime();
    public Hardware hardware;
    public Action time = telemetryPacket1 -> timer.milliseconds() > 1000;

    public MainActions(HardwareMap hardwareMap, Telemetry telemetry, Color a){
        hardware = new Hardware(hardwareMap, telemetry);
        PlaceOnRung = new ActionRunner(telemetry
        );
    }

    public ActionRunner PickUpOffWall;
    public ActionRunner PlaceOnRung;

}
