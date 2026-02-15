package org.firstinspires.ftc.teamcode;

import android.speech.RecognitionService;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.AutoLaunch;
import org.firstinspires.ftc.teamcode.Enums.BallColor;
import org.firstinspires.ftc.teamcode.Enums.Color;
import org.firstinspires.ftc.teamcode.Enums.Fire;
import org.firstinspires.ftc.teamcode.Enums.IntakeStates;
import org.firstinspires.ftc.teamcode.Enums.LaunchStates;

public class RobotContainer {

    public boolean isPressed = false;
    private boolean x = false;
    private boolean shootBasedOnOpenCVOrder;
    public Fire fireState = Fire.CannonOn;
    public Hardware hardware;
    public ActionRunner AutoRedFar;
    public ActionRunner AutoBlueFar;
    public boolean checkTag = false;
    public double[] Target;
    public ActionRunner AutoRedNear;
    public ActionRunner AutoBlueNear;
    public IntakeStates intakeStates;
    private ElapsedTime timer = new ElapsedTime();
    private int Counter = 0;
    public Action time = telemetryPacket1 -> timer.milliseconds() > 500;
    public Action resetTime = telemetryPacket -> {
        timer.reset();
        return true;
    };

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad){
        hardware = new Hardware(hardwareMap, telemetry);
        intakeStates = IntakeStates.Forward;
        AutoRedNear = new ActionRunner(telemetry
                , hardware.drive.Move(39, -0.4, telemetry)
                , hardware.drive.Turn(50, 0.4, telemetry)
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(-50, 0.4, telemetry)
                , hardware.cannon.cannonFireAuto(hardware,telemetry)
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
                , hardware.drive.Turn(-45, 0.25, telemetry)
                , hardware.drive.Move(15, 0.15,telemetry)
//                , IntakeThree(telemetry)
//                , hardware.drive.Move(24, -0.15, telemetry)
//                , hardware.drive.Turn(45, 0.25, telemetry)
//                , hardware.cannon.cannonFireAuto(hardware,telemetry)
//                , hardware.sorter.launch(hardware)
//                , hardware.cannon.cannonStop()
//                , hardware.drive.Strafe(10, -0.5, telemetry)
                , hardware.sorter.spinSorterToIntake(0)
                , hardware.drive.reset(telemetry)
                , hardware.sorter.reset()
        );
        AutoRedFar = new ActionRunner(telemetry
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(-50, 0.4, telemetry)
                , hardware.drive.AlignToTag(hardware, new double[]{38.6, 14.2, 64.51}, telemetry)
                , hardware.cannon.cannonFireAuto(hardware,telemetry)
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
                , hardware.drive.Move(8,0.1,telemetry)
                , hardware.sorter.spinSorterToIntake(0)
                , hardware.drive.reset(telemetry)
                , hardware.sorter.reset()
        );
        AutoBlueFar = new ActionRunner(telemetry
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(50, 0.4, telemetry)
                , hardware.drive.AlignToTag(hardware, new double[]{38.6, 14.2, 64.51}, telemetry)
                , hardware.cannon.cannonFireAuto(hardware,telemetry)
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
                , hardware.drive.Move(8,0.1,telemetry)
                , hardware.sorter.spinSorterToIntake(0)
                , hardware.drive.reset(telemetry)
                , hardware.sorter.reset()
        );
        AutoBlueNear = new ActionRunner(telemetry
                , hardware.drive.Move(39, -0.4, telemetry)
                , hardware.drive.Turn(-50, 0.4, telemetry)
                , hardware.camera.setOrderFromTag(telemetry)
                , hardware.drive.Turn(50, 0.4, telemetry)
                , hardware.cannon.cannonFireAuto(hardware,telemetry)
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
                , hardware.drive.Turn(45, 0.25, telemetry)
                , hardware.drive.Move(15, 0.15,telemetry)
                , IntakeThree(telemetry)
                , hardware.drive.Move(24, -0.15, telemetry)
                , hardware.drive.Turn(-45, 0.25, telemetry)
                , hardware.cannon.cannonFireAuto(hardware,telemetry)
                , hardware.sorter.launch(hardware)
                , hardware.cannon.cannonStop()
                , hardware.drive.Strafe(10, 0.5, telemetry)
                , hardware.sorter.spinSorterToIntake(0)
                , hardware.drive.reset(telemetry)
                , hardware.sorter.reset()
        );
    }

    public Action IntakeThree(Telemetry telemetry){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hardware.intake.Run().run(telemetryPacket);
                hardware.sorter.organizeSorter(telemetryPacket);
                switch(intakeStates){
                    case Forward:
                        if(hardware.drive.Move(3,0.1,telemetry).run(telemetryPacket)){
                            intakeStates = IntakeStates.Wait;
                            timer.reset();
                        }
                        break;
                    case Wait:
                        if(timer.milliseconds() > 400){
                            Counter++;
                            if(Counter >= 3){
                                intakeStates = IntakeStates.Reset;
                            } else{
                                intakeStates = IntakeStates.Forward;
                            }
                        }
                        break;
                    case Reset:
                        hardware.drive.reset(telemetry).run(telemetryPacket);
                        intakeStates = IntakeStates.Wait;
                        timer.reset();
                        if(Counter == 4){
                            hardware.intake.Stop().run(telemetryPacket);
                            Counter = 0;
                            intakeStates = IntakeStates.Forward;
                            return true;
                        }
                        break;
                }
                return false;
            }
        };
    }

    public void ControlCannon(Gamepad gamepad, Telemetry telemetry, TelemetryPacket packet, Color color){
        if(gamepad.a && !isPressed) {
            fireState = Fire.CannonOn;
            checkTag = false;
            isPressed = true;
        } else if(gamepad.right_bumper && !isPressed){
            fireState = Fire.AutoAlign;
            checkTag = false;
            isPressed = true;
        }
        if(isPressed){
            if(gamepad.x && fireState != Fire.Reset) {
                hardware.cannon.cannonStop().run(packet);
                hardware.sorter.sortMotor.moveWithPower(0);
                fireState = Fire.ResetKicker;
            }
            switch (fireState){
                case AutoAlign:
                    double[] target = hardware.camera.returnTarget(telemetry, color);
                    if((target == null || target.length == 0) && !checkTag){
                        hardware.sorter.autoLaunch = AutoLaunch.reset;
                        hardware.sorter.launchState = LaunchStates.MoveSort;
                        isPressed = false;
                        fireState = Fire.CannonOn;
                        hardware.drive.driveWithInput(0,0,0,telemetry);
                    } else{
                        Target = target;
                        checkTag = true;
                    }

                    if((target == null || target.length == 0)) {
                        hardware.sorter.autoLaunch = AutoLaunch.reset;
                        hardware.sorter.launchState = LaunchStates.MoveSort;
                        isPressed = false;
                        fireState = Fire.CannonOn;
                        hardware.drive.driveWithInput(0,0,0,telemetry);
                    }
                    else if(checkTag && hardware.drive.AlignToTag(hardware, Target, telemetry).run(new TelemetryPacket())) {
                        fireState = Fire.CannonOn;
                    }
                    break;
                case CannonOn:
                    hardware.cannon.cannonFire(hardware.camera.getCannonPower(telemetry)).run(packet);
                    fireState = Fire.Fire;
                    break;
                case Fire:
                    if(hardware.sorter.launch(hardware).run(packet)){
                        fireState = Fire.CannonOff;
                    }
                    break;
                case CannonOff:
                    hardware.cannon.cannonStop().run(packet);
                    isPressed = false;
                    fireState = Fire.AutoAlign;
                    break;
                case ResetKicker:
                    if(hardware.sorter.kicker.runToPosition(hardware.sorter.kicker.getPosition(),0,1)){
                        fireState = Fire.ResetSorter;
                    }
                    break;
                case ResetSorter:
                    if(hardware.sorter.spinSorterToIntake(0).run(packet)){
                        hardware.sorter.autoLaunch = AutoLaunch.reset;
                        hardware.sorter.launchState = LaunchStates.MoveSort;
                        isPressed = false;
                        fireState = Fire.AutoAlign;
                    }
                    break;
            }
        }
    }
    public void ControlReset(Gamepad gamepad, TelemetryPacket packet){
        if((gamepad.dpad_left || gamepad.dpad_right) && !hardware.intake.intakeRunning && !hardware.cannon.cannonFiring){
            if(gamepad.dpad_left) {
                hardware.sorter.sortMotor.moveWithPower(0.05);
                x = true;
            } else if(gamepad.dpad_right) {
                hardware.sorter.sortMotor.moveWithPower(-0.05);
                x = true;
            } else if(!isPressed){
                hardware.sorter.sortMotor.moveWithPower(0.0001);
                hardware.sorter.sortMotor.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                x = false;
            }
        } else if((hardware.sorter.isFull || !hardware.intake.intakeRunning) && !hardware.cannon.cannonFiring && !isPressed){
            hardware.sorter.sortMotor.moveWithPower(0.0001);
            if(x){
                hardware.sorter.sortMotor.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                x = false;
            }
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
