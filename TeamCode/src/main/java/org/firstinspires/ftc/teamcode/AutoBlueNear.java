package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.BallColor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoBlueNear extends LinearOpMode {
    RobotContainer robotContainer;

    @Override
    public void runOpMode() {
        robotContainer = new RobotContainer(hardwareMap,telemetry);
        robotContainer.hardware.sorter.holder[0] = BallColor.Green;
        robotContainer.hardware.sorter.holder[1] = BallColor.Purple;
        robotContainer.hardware.sorter.holder[2] = BallColor.Purple;
        robotContainer.hardware.sorter.isFull = true;
        waitForStart();
        while(opModeIsActive() && !robotContainer.AutoBlueNear.Run(new TelemetryPacket())){
            telemetry.addData("i",robotContainer.AutoBlueNear.index);
        }
        robotContainer.resetEverything();
    }
}
