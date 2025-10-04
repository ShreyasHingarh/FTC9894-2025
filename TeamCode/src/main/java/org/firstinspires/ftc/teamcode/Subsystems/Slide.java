package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.ServoWrapper;

@Config
public class Slide {
    public int trackedDegrees = 0;
    public DcMotorWrapper slide;
    public static int WALL = 350;
    public static int AboveWall = 800;
    public static int UP = 1500;
    public static int RUNGPOS = 760;

    public static double Kp = 0.005;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double SLIDE_VELOCITY = -0.9;


    public Slide(HardwareMap hardwareMap) {
        slide = new DcMotorWrapper(hardwareMap, "slide", Kp, Ki, Kd);
        slide.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void slideUp() {
        slide.MoveWithPID(UP);
    }
    public void slideDown() {
        slide.MoveWithPID(WALL);
    }
    public boolean canUp() {
        return true;
    }
    public boolean canDown() {
        return slide.getPosition() > 0;
    }

    public Action MoveToWall(double power){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (slide.moveMotorWithDeg(power, Math.abs(WALL))) {
                    return true;
                }
                return false;
            }
        };
    }
    public Action StopSlide(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slide.moveWithPower(0);
                return true;
            }
        };
    }
    public Action MoveAboveWall(double power) {
        return new ParallelAction(new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                trackedDegrees = AboveWall;
                return slide.moveMotorWithDeg(power,AboveWall);
            }
        });
    }
    public Action MoveDownToWall(double power) {
        return new ParallelAction(new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (slide.moveMotorWithDeg(power, Math.abs(trackedDegrees))) {
                    trackedDegrees = 0;
                    return true;
                }
                return false;
            }
        });
    }
    public Action MoveUpToPlace(double power){
        return new ParallelAction(new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(trackedDegrees == UP) return true;
                if(slide.moveMotorWithDeg(power,Math.abs(UP - trackedDegrees))){
                    trackedDegrees = UP;
                    return true;
                }
                return false;
            }
        });
    }
    public void controllingSlide(Gamepad gamepad2) {
        //slide logic
        if (gamepad2.left_stick_y < 0) {
            if (!canUp()) {
                slide.moveWithPower(0);
            }
            else {
                slide.moveWithPower(gamepad2.left_stick_y * Slide.SLIDE_VELOCITY);
            }
        } else if (gamepad2.left_stick_y > 0) {
            if (!canDown()) {
                slide.moveWithPower(0);
            }
            else {
                slide.moveWithPower(-0.25);//gamepad2.left_stick_y * Slide.SLIDE_VELOCITY);
            }
        }
        else {
            slide.moveWithPower(0);
        }

    }
}

