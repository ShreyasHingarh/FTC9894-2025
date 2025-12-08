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
    RobotContainer mainActions;

    @Override
    public void runOpMode() {
        mainActions = new RobotContainer(hardwareMap,telemetry, Color.Blue);
        mainActions.hardware.sorter.holder[0] = BallColor.Green;
        mainActions.hardware.sorter.holder[1] = BallColor.Purple;
        mainActions.hardware.sorter.holder[2] = BallColor.Purple;
        mainActions.hardware.sorter.isFull = true;
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        mainActions.hardware.sorter.sortMotor.motor.setTargetPosition(0);
        mainActions.hardware.sorter.sortMotor.motor.setPower(0.2);
        mainActions.hardware.sorter.sortMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mainActions.hardware.sorter.sortMotor.getPosition() != 0){
            mainActions.hardware.camera.telemetryAprilTag(telemetry);
        }
        while(!mainActions.hardware.drive.Move(42, -0.5, telemetry).run(new TelemetryPacket())) {
            mainActions.hardware.camera.telemetryAprilTag(telemetry);
        }
        Actions.runBlocking(
                mainActions.hardware.drive.Turn(-60, 0.15, telemetry)
        );
        while(!mainActions.hardware.camera.setOrderFromTag(telemetry)){
            telemetry.addData("a",mainActions.hardware.camera.telemetryAprilTag(telemetry)[0]);
            telemetry.update();
        }
        Actions.runBlocking(
                mainActions.hardware.drive.Turn(60, 0.15, telemetry)
        );
        mainActions.hardware.cannon.cannon.moveWithPower(-1);
        while(!mainActions.hardware.sorter.launch(mainActions.hardware.camera.Order).run(new TelemetryPacket())){

        }
        time.reset();
        mainActions.hardware.drive.driveWithInput(-0.5,0,0,telemetry);
        while(time.milliseconds() < 1000){

        }
        mainActions.hardware.drive.driveWithInput(0,0,0,telemetry);
        while(!mainActions.hardware.sorter.spinSorterToIntake(0).run(new TelemetryPacket())){

        }
        mainActions.resetEverything();
    }
}
