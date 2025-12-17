package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.ServoWrapper;

@TeleOp
public class BlockArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor sensor;
        sensor = hardwareMap.get(ColorSensor.class,"color3");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("r", sensor.red());
            telemetry.addData("g", sensor.green());
            telemetry.addData("b", sensor.blue());
            telemetry.update();
        }
    }
}
