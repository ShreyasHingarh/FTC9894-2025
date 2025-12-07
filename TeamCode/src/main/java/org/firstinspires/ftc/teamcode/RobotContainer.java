package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.Color;

public class RobotContainer {

    public boolean isPressed = false;
    public boolean resetFromCannonFiring = true;
    public boolean resetFromLaunching = true;
    public boolean hasReset = false;
    public Hardware hardware;
    public ActionRunner AutoRed;
    public ActionRunner AutoBlue;
    //private ElapsedTime timer = new ElapsedTime();
    //public Action time = telemetryPacket1 -> timer.milliseconds() > 1000;

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, Color a){
        hardware = new Hardware(hardwareMap, telemetry);
        AutoRed = new ActionRunner(telemetry
                , hardware.drive.Move(40,0.4,telemetry));
        AutoBlue = new ActionRunner(telemetry
                , hardware.drive.Move(40,0.4,telemetry));
    }


    public void ControlCannon(Gamepad gamepad, TelemetryPacket packet){
        hardware.cannon.controllingCannon(gamepad, packet);
        if(hardware.cannon.cannonFiring){
            resetFromCannonFiring = false;
            if(gamepad.x){
                if(!isPressed && resetFromLaunching){
                    isPressed = true;
                }
//                else {
//                    isPressed = false;
//                    resetFromLaunching = false;
//                }
            }
            if(isPressed) {
                isPressed = !hardware.sorter.launch(hardware.camera.Order).run(packet);
            }
        }
//        else if(!resetFromCannonFiring){
//            resetFromCannonFiring = true;//resetFromCannonFiring = hardware.sorter.reset().run(packet);
//        }
//
//        if(!resetFromLaunching){
//            resetFromLaunching = true;
//            //resetFromLaunching = hardware.sorter.reset().run(packet);
//        }
    }
    public void ControlReset(Gamepad gamepad, TelemetryPacket packet){
        if(gamepad.dpad_up && !hasReset){
            hasReset = true;
        }
        if(hasReset){
            hasReset = false;
            //hasReset = !hardware.sorter.reset().run(packet);
        }
    }
    public void ControlSort(Gamepad gamepad, TelemetryPacket packet){
        if(hasReset || hardware.cannon.cannonFiring || !resetFromCannonFiring || !resetFromLaunching) {
            hardware.intake.Stop().run(packet);
            return;
        }
        hardware.intake.control(gamepad,packet, hardware.sorter.isFull);
        if(!hardware.sorter.isFull && hardware.intake.intakeRunning) {
            hardware.sorter.organizeSorter(packet);
        }
    }
//    public void ControlCannon(Gamepad gamepad, TelemetryPacket packet) {
//        hardware.cannon.controllingCannon(gamepad, packet);
//        //when firing
//        if (hardware.cannon.cannonFiring) {
//            //manages check for shifting to launch position
//            if (offsetState == OffsetState.Stop && !a) {
//                offsetState = OffsetState.MoveToOffset;
//            }
//            //check if need to launch
//            if(colorToShoot == BallColor.None){
//                if(gamepad.x){
//                    colorToShoot = BallColor.Green;
//                } else if (gamepad.y) {
//                    colorToShoot = BallColor.Purple;
//                }
//            }
//            else{
//                colorToShoot = hardware.sorter.launchAtColor(colorToShoot).run(packet) ? BallColor.None : colorToShoot;
//            }
//        } else if (offsetState == OffsetState.Stop && a) {//If no longer launching anymore reset
//            offsetState = OffsetState.ResetOffset;
//        }
//
//        //manages states for Offset
//        switch(offsetState){
//            case MoveToOffset:
//                offsetState = hardware.sorter.moveOffSet().run(packet) ? OffsetState.Stop : offsetState;
//                a = true;
//                break;
//            case ResetOffset:
//                offsetState = hardware.sorter.resetOffset().run(packet) ? OffsetState.Stop : offsetState;
//                a = false;
//                break;
//        }
//    }
}
