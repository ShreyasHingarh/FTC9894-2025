package org.firstinspires.ftc.teamcode.TestModes;

import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Enums.LaunchStates;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Cannon;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.ServoWrapper;

@TeleOp
public class BlockArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorWrapper motor;
        motor = new DcMotorWrapper(hardwareMap, "kicker",0,0,0);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        boolean x = false;
        boolean y = false;
        ElapsedTime timer = new ElapsedTime();
        LaunchStates states = LaunchStates.wait;
        while(opModeIsActive()){
            //            telemetry.addData("red",motor.red());
//            telemetry.addData("b",motor.blue());
//            telemetry.addData("g",motor.green());
//            telemetry.addData("hue", (int) JavaUtil.colorToHue((Color.rgb(motor.red(), motor.green(),motor.blue()))));
//            telemetry.update();
            telemetry.addData("pos", motor.getPosition());
            telemetry.update();

            if(gamepad2.x && !x){
                x = true;
                timer.reset();
            }

            if (x){
                switch(states){
                    case wait:
                        timer.reset();
                        states = LaunchStates.kickerLaunch;
                        break;
                    case kickerLaunch:
                        motor.moveWithPower(1);
                        if(timer.milliseconds() > 300){
                            timer.reset();
                            states = LaunchStates.kickerReset;
                        }
                        break;
                    case kickerReset:
                        motor.moveWithPower(-1);
                        if(timer.milliseconds() > 300){
                            motor.moveWithPower(0);
                            timer.reset();
                            states = LaunchStates.wait;
                            x = false;
                        }
                        break;
                }
            }
        }
    }
}
