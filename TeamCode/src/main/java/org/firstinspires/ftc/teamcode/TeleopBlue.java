package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.AutoLaunch;
import org.firstinspires.ftc.teamcode.Enums.Color;
import org.firstinspires.ftc.teamcode.Enums.Fire;
import org.firstinspires.ftc.teamcode.Enums.LaunchStates;

@TeleOp
public class TeleopBlue extends LinearOpMode {
    RobotContainer robotContainer;
    TelemetryPacket pack;

    /// Integrate it into the rest of the code (fix the isorter order) (add functionality to run either order)
    /// dynamic cannon power using april tag
    /// Align to tag from far away
    /// create the paths for the auto
    private void showTelemetry(){
        telemetry.addData("degrees", robotContainer.hardware.sorter.sortMotor.getPosition());
        telemetry.addData("cannonFiring?", robotContainer.hardware.cannon.cannonFiring);
        telemetry.addData("order 1", robotContainer.hardware.camera.Order[0]);
        telemetry.addData("order 2", robotContainer.hardware.camera.Order[1]);
        telemetry.addData("order 3", robotContainer.hardware.camera.Order[2]);
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
        telemetry.addData("color 3 hue",c[3]);

        telemetry.addData("Color 1", robotContainer.hardware.sorter.sensorSeesBall(robotContainer.hardware.sorter.color1));
        telemetry.addData("Color 2", robotContainer.hardware.sorter.sensorSeesBall(robotContainer.hardware.sorter.color2));
        telemetry.addData("isFull", robotContainer.hardware.sorter.isFull);
        telemetry.addData("currentPosition", robotContainer.hardware.sorter.currentPosition);
        telemetry.addData("sorterPower", robotContainer.hardware.sorter.sortMotor.getPower());
        telemetry.addData("hold 0", robotContainer.hardware.sorter.holder[0]);
        telemetry.addData("hold 1", robotContainer.hardware.sorter.holder[1]);
        telemetry.addData("hold 2", robotContainer.hardware.sorter.holder[2]);
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robotContainer = new RobotContainer(hardwareMap,telemetry);
        double reduction = 0.2;
        waitForStart();
        while(!robotContainer.hardware.camera.PickOrder(gamepad2)){

        }
        while(gamepad2.x || gamepad2.b || gamepad2.a){

        }
        while (opModeIsActive()) {
            robotContainer.ControlSort(gamepad2, pack);
            robotContainer.ControlCannon(gamepad2, telemetry, pack, Color.Blue);
            robotContainer.ControlReset(gamepad2, pack);
            if(gamepad2.left_bumper && !robotContainer.checkTag){
                robotContainer.hardware.drive.driveWithInput(reduction * gamepad2.left_stick_x, reduction * -gamepad2.left_stick_y, reduction * gamepad2.right_stick_x, telemetry);
            }
            else if(!robotContainer.checkTag){
                robotContainer.hardware.drive.driveWithInput(gamepad2.left_stick_x, -gamepad2.left_stick_y, 0.7 * gamepad2.right_stick_x, telemetry);
            }

            if(robotContainer.checkTag && (gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0)){
                robotContainer.checkTag = false;
                robotContainer.hardware.sorter.autoLaunch = AutoLaunch.reset;
                robotContainer.hardware.sorter.launchState = LaunchStates.MoveSort;
                robotContainer.isPressed = false;
                robotContainer.fireState = Fire.CannonOn;
            }
            showTelemetry();
        }
        robotContainer.resetEverything();
    }
}
