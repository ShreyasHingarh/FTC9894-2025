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
    public Hardware hardware;
    public ActionRunner AutoRed;
    public ActionRunner AutoBlue;
    private ElapsedTime timer = new ElapsedTime();
    public Action time = telemetryPacket1 -> timer.milliseconds() > 5000;
    public Action resetTime = telemetryPacket -> {
        timer.reset();
        return true;
    };

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, Color a){
        hardware = new Hardware(hardwareMap, telemetry);
        AutoRed = new ActionRunner(telemetry
                //, hardware.sorter.reset()
                , hardware.drive.Move(35, -0.5, telemetry)
                , hardware.drive.Turn(60, 0.25, telemetry)
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(-45, 0.25, telemetry)
                //
//                ,time
                , hardware.cannon.cannonFire()
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
//                , hardware.drive.Move(5,-0.5,telemetry) //here
//                , hardware.drive.Turn(40,0.25,telemetry) //here
                // intake three
                // invert driving from the here lines
                 //,hardware.drive.AlignToTag(hardware, telemetry)
//                , hardware.cannon.cannonFire()
//                , hardware.sorter.launch(hardware)
//                , hardware.cannon.cannonStop()
                , hardware.drive.Strafe(15,-0.5,telemetry)
                , hardware.sorter.spinSorterToIntake(0)
                , hardware.drive.reset(telemetry)
                , hardware.sorter.reset()
        );

        AutoBlue = new ActionRunner(telemetry
                , hardware.sorter.reset()
                , hardware.drive.Move(35, -0.5, telemetry)
                , hardware.drive.Turn(-60, 0.25, telemetry)
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(70, 0.25, telemetry)
                //
//                ,time
                , hardware.cannon.cannonFire()
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
                , hardware.drive.Strafe(15,0.5,telemetry)
                , hardware.sorter.spinSorterToIntake(0)
                , hardware.drive.reset(telemetry)
                , hardware.sorter.reset());
    }

    public Action IntakeThree(Telemetry telemetry){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hardware.drive.driveWithInput(0,0.1,0, telemetry);
                hardware.intake.Run().run(telemetryPacket);
                if(!hardware.sorter.isFull) {
                    hardware.sorter.organizeSorter(telemetryPacket);
                    return false;
                }
                hardware.drive.reset(telemetry).run(telemetryPacket);
                hardware.intake.Stop().run(telemetryPacket);
                return true;
            }
        };
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
            }
        }
    }
    boolean x = false;
    public void ControlReset(Gamepad gamepad, TelemetryPacket packet){
        if(gamepad.dpad_left || gamepad.dpad_right){
            if(gamepad.dpad_left) {
                hardware.sorter.sortMotor.moveWithPower(0.05);
                x = true;
            } else if(gamepad.dpad_right) {
                hardware.sorter.sortMotor.moveWithPower(-0.05);
                x = true;
            }
        } else if((hardware.sorter.isFull || !hardware.intake.intakeRunning) && !isPressed){
            hardware.sorter.sortMotor.moveWithPower(0.0001);
            if(x){
                hardware.sorter.sortMotor.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                x = false;
            }
        }

        if(gamepad.dpad_down){
            hardware.sorter.servo.setPosition(hardware.sorter.SERVOPOSITION);
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
