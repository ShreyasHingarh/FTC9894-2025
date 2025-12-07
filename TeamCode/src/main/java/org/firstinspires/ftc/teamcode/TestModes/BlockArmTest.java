package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;

@TeleOp
public class BlockArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorWrapper motor = new DcMotorWrapper(hardwareMap,"sorter",0,0,0);
        waitForStart();
        while(opModeIsActive()){
            if(motor.moveMotorWithDeg(0.2,45)){
                break;
            }
        }
    }
}
