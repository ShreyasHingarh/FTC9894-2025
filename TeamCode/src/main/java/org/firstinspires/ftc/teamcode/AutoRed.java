package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.BallColor;
import org.firstinspires.ftc.teamcode.Enums.Color;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.Node;

@Autonomous
public class AutoRed extends LinearOpMode {
    RobotContainer mainActions;
    //move out
    //see pattern to shoot
    //shoot the 3
    //intake next 3
    //shoot the next 3 aligning to apriltag
    //intake next 3
    //shoot the next 3 aligning to apriltag
    //move out

    @Override
    public void runOpMode() throws InterruptedException {
        mainActions = new RobotContainer(hardwareMap,telemetry, Color.Blue);
        mainActions.hardware.sorter.holder[0] = BallColor.Purple;
        mainActions.hardware.sorter.holder[1] = BallColor.Green;
        mainActions.hardware.sorter.holder[2] = BallColor.Purple;
        mainActions.hardware.sorter.isFull = true;
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        mainActions.hardware.sorter.sortMotor.motor.setTargetPosition(0);
        mainActions.hardware.sorter.sortMotor.motor.setPower(0.2);
        mainActions.hardware.sorter.sortMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mainActions.hardware.sorter.sortMotor.getPosition() != 0){

        }
        while(!mainActions.hardware.drive.Move(40, -0.5, telemetry).run(new TelemetryPacket())) {

        }
//        mainActions.hardware.cannon.cannonFire().run(new TelemetryPacket());
//        while(!mainActions.hardware.sorter.launchAtColor(BallColor.Purple).run(new TelemetryPacket())){
//
//        }
//        while(!mainActions.hardware.sorter.launchAtColor(BallColor.Green).run(new TelemetryPacket())){
//
//        }
//        while(!mainActions.hardware.sorter.launchAtColor(BallColor.Purple).run(new TelemetryPacket())) {
//
//        }
        mainActions.hardware.sorter.sortMotor.motor.setTargetPosition(0);
        mainActions.hardware.sorter.sortMotor.motor.setPower(0.2);
        mainActions.hardware.sorter.sortMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mainActions.hardware.sorter.sortMotor.getPosition() != 0){

        }
        time.reset();
        mainActions.hardware.drive.driveWithInput(-0.5,0,0,telemetry);
        while(time.milliseconds() < 1250){

        }
        mainActions.hardware.drive.driveWithInput(0,0,0,telemetry);
    }
}
