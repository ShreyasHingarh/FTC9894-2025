package org.firstinspires.ftc.teamcode.Subsystems;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;

public class Intake {
    public DcMotorWrapper Intake;
    public final double INTAKE_VELOCITY = 1;
    private boolean isPressed = false;
    private boolean isYPressed = false;
    public boolean outtakeRunning = false;
    public boolean intakeRunning = false;

    public Intake(HardwareMap hardwareMap){
        Intake = new DcMotorWrapper(hardwareMap, "intake", 0,0,0);
        Intake.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public Action Run() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Intake.moveWithPower(INTAKE_VELOCITY);
                intakeRunning = true;
                return true;
            }
        };
    }
    public Action Outake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Intake.moveWithPower(-INTAKE_VELOCITY);
                outtakeRunning = true;
                return true;
            }
        };
    }
    public Action Stop() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Intake.moveWithPower(0);
                intakeRunning = false;
                outtakeRunning = false;
                return true;
            }
        };
    }

    public void control(Gamepad gamepad2, TelemetryPacket packet, boolean isFull) {
        //intake logic
        if(!isFull) {
            if (gamepad2.b && !isPressed) {
                if (!intakeRunning) {
                    Run().run(packet);
                    outtakeRunning = false;
                } else {
                    Stop().run(packet);
                }
                isPressed = true;
            } else if (!gamepad2.b && isPressed) {
                isPressed = false;
            }

            if (gamepad2.y && !isYPressed) {
                if (!outtakeRunning) {
                    Outake().run(packet);
                    intakeRunning = false;
                } else {
                    Stop().run(packet);
                }
                isYPressed = true;
            } else if (!gamepad2.y && isYPressed) {
                isYPressed = false;
            }
        } else{
            isPressed = false;
            intakeRunning = false;
            Stop().run(packet);
        }
    }
}
