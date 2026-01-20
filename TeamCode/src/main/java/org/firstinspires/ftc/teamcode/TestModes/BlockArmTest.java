package org.firstinspires.ftc.teamcode.TestModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.ServoWrapper;

@TeleOp
public class BlockArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorWrapper motor;
        motor = new DcMotorWrapper(hardwareMap,"kicker",0,0,0);
        motor.ResetMotor();
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while(opModeIsActive()){
//            telemetry.addData("red",motor.red());
//            telemetry.addData("b",motor.blue());
//            telemetry.addData("g",motor.green());
//            telemetry.addData("hue", (int) JavaUtil.colorToHue((Color.rgb(motor.red(), motor.green(),motor.blue()))));
//            telemetry.update();
            telemetry.addData("pos", motor.getPosition());
            telemetry.update();
        }
    }
}
