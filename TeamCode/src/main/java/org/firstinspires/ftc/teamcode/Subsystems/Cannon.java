package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;

public class Cannon {

    public DcMotorWrapper cannon;
    public static double CANNON_VELOCITY = -0.9;
    public boolean cannonFiring;
    public Cannon(HardwareMap hardwareMap) {
        cannon = new DcMotorWrapper(hardwareMap, "cannon", 0,0,0);
        cannon.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cannon.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cannon.setDirection(DcMotorSimple.Direction.FORWARD);
        cannonFiring = false;
    }


    public Action cannonFire() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                cannon.moveWithPower(CANNON_VELOCITY);
                cannonFiring = true;
                return true;
            }
        };
    }
    public Action cannonStop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                cannon.moveWithPower(0);
                cannonFiring = false;
                return true;
            }
        };
    }
}

