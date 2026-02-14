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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;

public class Cannon {

    public DcMotorWrapper cannon;
    public boolean cannonFiring;
    public Cannon(HardwareMap hardwareMap) {
        cannon = new DcMotorWrapper(hardwareMap, "cannon", 0,0,0);
        cannon.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cannon.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cannon.setDirection(DcMotorSimple.Direction.REVERSE);
        cannonFiring = false;
    }


    public Action cannonFire(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                cannon.moveWithPower(power);
                cannonFiring = true;
                return true;
            }
        };
    }
    public Action cannonFireAuto(Hardware hardware, Telemetry telemetry) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double x = hardware.camera.getCannonPower(telemetry);
                cannon.moveWithPower(x);
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

