package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums.BallColor;

@Autonomous
public class AutoRedNear extends LinearOpMode {

    RobotContainer robotContainer;
    //move out
    //see pattern to shoot
    //shoot the 3
    //intake next 3
    //shoot the next 3 aligning to apriltag
    //intake next 3
    //shoot the next 3 aligning to apriltag
    //move out
    private void showTelemetry(){
        telemetry.addData("isPressed", robotContainer.isPressed);
        telemetry.addData("LaunchState", robotContainer.hardware.sorter.launchState);
        telemetry.addData("AutoLaunchState", robotContainer.hardware.sorter.autoLaunch);
        telemetry.addData("indexToSpinTo", robotContainer.hardware.sorter.indexToSpinTo);
        telemetry.addData("currentPos", robotContainer.hardware.sorter.currentDegrees);
        telemetry.addData("orderToSpin 1", robotContainer.hardware.sorter.orderToLaunch[0]);
        telemetry.addData("orderToSpin 2", robotContainer.hardware.sorter.orderToLaunch[1]);
        telemetry.addData("orderToSpin 3", robotContainer.hardware.sorter.orderToLaunch[2]);
        int[] a = robotContainer.hardware.sorter.getSensorValue(robotContainer.hardware.sorter.color1);
        telemetry.addData("color 1 r",a[0]);
        telemetry.addData("color 1 g",a[1]);
        telemetry.addData("color 1 b",a[2]);
        telemetry.addData("color 1 a",a[3]);
        int[] b = robotContainer.hardware.sorter.getSensorValue(robotContainer.hardware.sorter.color2);
        telemetry.addData("color 2 r",b[0]);
        telemetry.addData("color 2 g",b[1]);
        telemetry.addData("color 2 b",b[2]);
        telemetry.addData("color 2 a",b[3]);

        int[] c = robotContainer.hardware.sorter.getSensorValue(robotContainer.hardware.sorter.color3);
        telemetry.addData("color 3 r",c[0]);
        telemetry.addData("color 3 g",c[1]);
        telemetry.addData("color 3 b",c[2]);
        telemetry.addData("color 3 a",c[3]);

        telemetry.addData("Color 1", robotContainer.hardware.sorter.sensorSeesBall(robotContainer.hardware.sorter.color1));
        telemetry.addData("Color 2", robotContainer.hardware.sorter.sensorSeesBall(robotContainer.hardware.sorter.color2));
        telemetry.addData("isFull", robotContainer.hardware.sorter.isFull);
        telemetry.addData("currentPosition", robotContainer.hardware.sorter.currentPosition);
        telemetry.addData("sorterPower", robotContainer.hardware.sorter.sortMotor.getPower());
        telemetry.addData("hold 0", robotContainer.hardware.sorter.holder[0]);
        telemetry.addData("hold 1", robotContainer.hardware.sorter.holder[1]);
        telemetry.addData("hold 2", robotContainer.hardware.sorter.holder[2]);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robotContainer = new RobotContainer(hardwareMap,telemetry);
        robotContainer.hardware.sorter.holder[0] = BallColor.Green;
        robotContainer.hardware.sorter.holder[1] = BallColor.Purple;
        robotContainer.hardware.sorter.holder[2] = BallColor.Purple;
        robotContainer.hardware.sorter.isFull = true;
        waitForStart();
        while(opModeIsActive() && !robotContainer.AutoRedNear.Run(new TelemetryPacket())){
            telemetry.addData("ticks", robotContainer.hardware.drive.rightFront.getCurrentPosition());
            telemetry.addData("i",robotContainer.AutoRedNear.index);
            telemetry.addData("power", robotContainer.hardware.cannon.cannon.getPower());
        }
        robotContainer.resetEverything();
    }
}
