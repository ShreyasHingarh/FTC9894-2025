package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.Color;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoBlue extends LinearOpMode {
    MainActions mainActions;

    @Override
    public void runOpMode() {
        mainActions = new MainActions(hardwareMap,telemetry, Color.Blue);
        waitForStart();
        while(opModeIsActive()){

        }
    }
}
