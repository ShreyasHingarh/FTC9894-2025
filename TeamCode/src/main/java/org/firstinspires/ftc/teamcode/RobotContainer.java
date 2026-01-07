package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.Color;

public class RobotContainer {

    public boolean isPressed = false;
    public boolean hasReset = false;
    public Hardware hardware;
    public ActionRunner AutoRed;
    public ActionRunner AutoBlue;
    private ElapsedTime timer = new ElapsedTime();
    public Action time = telemetryPacket1 -> timer.milliseconds() > 5000;

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, Color a){
        hardware = new Hardware(hardwareMap, telemetry);
        AutoRed = new ActionRunner(telemetry
                , hardware.sorter.reset()
                , hardware.drive.Move(35, -0.5, telemetry)
                , hardware.drive.Turn(60, 0.25, telemetry)
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(-60, 0.25, telemetry)
//                ,time
//                , hardware.cannon.cannonFire()
//                , hardware.sorter.launch(hardware)
//                , hardware.cannon.cannonStop()
                , hardware.drive.Move(5,-0.5,telemetry)
                , hardware.drive.Turn(40,0.25,telemetry)
//                , hardware.drive.Strafe(15,0.5,telemetry)
//                , hardware.sorter.spinSorterToIntake(0)
//                , hardware.drive.reset(telemetry)
//                , hardware.sorter.reset()
        );

        AutoBlue = new ActionRunner(telemetry
                , hardware.sorter.reset()
                , hardware.drive.Move(44, -0.5, telemetry)
                , hardware.drive.Turn(60, 0.15, telemetry)
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(-60, 0.15, telemetry)
                , hardware.cannon.cannonFire()
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
                , hardware.drive.Strafe(15,-0.5,telemetry)
                , hardware.sorter.spinSorterToIntake(0)
                , hardware.drive.reset(telemetry)
                , hardware.sorter.reset());
    }


    public void ControlCannon(Gamepad gamepad, TelemetryPacket packet){
        hardware.cannon.controllingCannon(gamepad, packet);
        if(hardware.cannon.cannonFiring){
            if(gamepad.x){
                if(!isPressed){
                    isPressed = true;
                }
            }

            if(isPressed) {
                isPressed = !hardware.sorter.launch(hardware).run(packet);
            } else{
                hardware.sorter.sortMotor.runToPosition(hardware.sorter.sortMotor.getPosition(),hardware.sorter.sortPosition,0.1);
            }
        }
    }
    public void ControlReset(Gamepad gamepad, TelemetryPacket packet){
        if(gamepad.dpad_up && !hasReset){
            hardware.sorter.sortMotor.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.sorter.sortMotor.moveWithPower(0.1);
            hasReset = true;
        }
        else if(hasReset){
            hasReset = !hardware.sorter.reset().run(packet);
        }

        if(gamepad.dpad_down){
            hardware.sorter.servo.setPosition(0.48);
        }
    }
    public void ControlSort(Gamepad gamepad, TelemetryPacket packet){
        hardware.intake.control(gamepad,packet, hardware.sorter.isFull);
        if(!hardware.sorter.isFull && hardware.intake.intakeRunning) {
            hardware.sorter.organizeSorter(packet);
        }
    }
    public void resetEverything(){
        hardware.sorter.sortMotor.moveWithPower(0);
        hardware.cannon.cannon.moveWithPower(0);
        hardware.intake.Intake.moveWithPower(0);
        hardware.drive.MoveChassisWithPower(0,0,0,0);

    }
}
