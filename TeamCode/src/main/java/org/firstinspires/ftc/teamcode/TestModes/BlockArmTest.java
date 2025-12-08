package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.ServoWrapper;

@TeleOp
public class BlockArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo motor = hardwareMap.get(Servo.class,"kickerReal");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                motor.setPosition(0.2);
            } else if(gamepad1.b){
                motor.setPosition(0.52);
            } else if(gamepad1.x){
                motor.setPosition(0.8);
            }
        }
    }
}
