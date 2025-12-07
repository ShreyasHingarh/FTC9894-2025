package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.Color;

@TeleOp
public class Teleop extends LinearOpMode {
    RobotContainer robotContainer;
    TelemetryPacket pack;

    private void showTelemetry(){
        telemetry.addData("degrees", robotContainer.hardware.sorter.sortMotor.getPosition());
        telemetry.addData("cannonFiring?", robotContainer.hardware.cannon.cannonFiring);
        telemetry.addData("isPressed", robotContainer.isPressed);
        telemetry.addData("LaunchState", robotContainer.hardware.sorter.launchState);
        telemetry.addData("AutoLaunchState", robotContainer.hardware.sorter.autoLaunch);
        telemetry.addData("indexToSpinTo", robotContainer.hardware.sorter.indexToSpinTo);
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
        telemetry.addData("Color 1", robotContainer.hardware.sorter.sensorSeesBall(robotContainer.hardware.sorter.color1));
        telemetry.addData("Color 2", robotContainer.hardware.sorter.sensorSeesBall(robotContainer.hardware.sorter.color2));
        telemetry.addData("isFull", robotContainer.hardware.sorter.isFull);
        telemetry.addData("currentPosition", robotContainer.hardware.sorter.currentPosition);
        telemetry.addData("hold 0", robotContainer.hardware.sorter.holder[0]);
        telemetry.addData("hold 1", robotContainer.hardware.sorter.holder[1]);
        telemetry.addData("hold 2", robotContainer.hardware.sorter.holder[2]);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robotContainer = new RobotContainer(hardwareMap,telemetry,Color.Red);
//        robotContainer.hardware.sorter.reset().run(pack);
        waitForStart();
        while(!robotContainer.hardware.camera.PickOrder(gamepad2)){
            showTelemetry();
            telemetry.update();
        }
        while(gamepad2.x || gamepad2.b || gamepad2.a){
            showTelemetry();
            telemetry.update();
        }
        while(opModeIsActive()) {
            robotContainer.ControlSort(gamepad2,pack);
            robotContainer.ControlCannon(gamepad2,pack);
            robotContainer.ControlReset(gamepad2,pack);
            //DRIVE
            robotContainer.hardware.drive.driveWithInput(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, telemetry);
            showTelemetry();
            telemetry.update();
        }
    }
}
