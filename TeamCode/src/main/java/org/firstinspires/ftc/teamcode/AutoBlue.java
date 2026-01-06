package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.BallColor;
import org.firstinspires.ftc.teamcode.Enums.Color;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoBlue extends LinearOpMode {
    RobotContainer robotContainer;

    @Override
    public void runOpMode() {
        robotContainer = new RobotContainer(hardwareMap,telemetry, Color.Blue);
        robotContainer.hardware.sorter.holder[0] = BallColor.Green;
        robotContainer.hardware.sorter.holder[1] = BallColor.Purple;
        robotContainer.hardware.sorter.holder[2] = BallColor.Purple;
        robotContainer.hardware.sorter.isFull = true;
        waitForStart();
        while(!robotContainer.AutoBlue.Run(new TelemetryPacket())){

        }
        robotContainer.resetEverything();
    }
}
