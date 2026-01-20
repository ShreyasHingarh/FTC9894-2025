package org.firstinspires.ftc.teamcode.TestModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp
public class TestSort extends LinearOpMode {
    DcMotor motor;
    public boolean runToPosition(int currentPosition, int position, double power) {
        motor.setTargetPosition(position);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return currentPosition == position;
    }
    public int returnRangedDegrees(){
        int i = motor.getCurrentPosition();
        while(i >= 540){
            i -= 540;
        }
        while(i < 0){
            i += 540;
        }
        return i;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class,"sorter");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime time = new ElapsedTime();

        waitForStart();
        time.reset();
        for(int i = 0 ;i < 10;i++) {
            while(opModeIsActive() && !runToPosition(returnRangedDegrees(),180,0.2)){
                telemetry.addData("current Position",returnRangedDegrees());
                telemetry.update();
            }
            time.reset();
            while(time.milliseconds() < 3000) {

            }
            while(opModeIsActive() && !runToPosition(returnRangedDegrees(),360,0.2)){
                telemetry.addData("current Position",returnRangedDegrees());
                telemetry.update();
            }
            time.reset();
            while(time.milliseconds() < 3000) {

            }
            while(opModeIsActive() && !runToPosition(returnRangedDegrees(),0,0.2)){
                telemetry.addData("current Position",returnRangedDegrees());
                telemetry.update();
            }
            time.reset();
            while(time.milliseconds() < 3000) {

            }
            while(opModeIsActive() && !runToPosition(returnRangedDegrees(),90,0.2)){
                telemetry.addData("current Position",returnRangedDegrees());
                telemetry.update();
            }
            time.reset();
            while(time.milliseconds() < 3000) {

            }
            while(opModeIsActive() && !runToPosition(returnRangedDegrees(),270,0.2)){
                telemetry.addData("current Position",returnRangedDegrees());
                telemetry.update();
            }
            time.reset();
            while(time.milliseconds() < 3000) {

            }
            while(opModeIsActive() && !runToPosition(returnRangedDegrees(),450,0.2)){
                telemetry.addData("current Position",returnRangedDegrees());
                telemetry.update();
            }
            time.reset();
            while(time.milliseconds() < 3000) {

            }
        }
    }
}
