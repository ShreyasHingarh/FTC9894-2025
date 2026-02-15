package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.BallColor;

@Autonomous
public class AutoBlueNear extends LinearOpMode {

    RobotContainer robotContainer;
    @Override
    public void runOpMode() throws InterruptedException {
        robotContainer = new RobotContainer(hardwareMap,telemetry);
        robotContainer.hardware.sorter.holder[0] = BallColor.Green;
        robotContainer.hardware.sorter.holder[1] = BallColor.Purple;
        robotContainer.hardware.sorter.holder[2] = BallColor.Purple;
        robotContainer.hardware.sorter.isFull = true;
        waitForStart();
        while(opModeIsActive() && !robotContainer.AutoBlueNear.Run(new TelemetryPacket())){
            telemetry.addData("ticks", robotContainer.hardware.drive.rightFront.getCurrentPosition());
            telemetry.addData("i",robotContainer.AutoBlueNear.index);
            telemetry.addData("power", robotContainer.hardware.cannon.cannon.getPower());
        }
        robotContainer.resetEverything();
    }
}