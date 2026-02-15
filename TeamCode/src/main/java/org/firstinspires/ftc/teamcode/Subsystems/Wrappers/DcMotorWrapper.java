package org.firstinspires.ftc.teamcode.Subsystems.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DcMotorWrapper {
    private boolean Reset = false;
    public DcMotorEx motor;
    private ElapsedTime timer;
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private double lastError;
    private double integralSum;

    public DcMotorWrapper(HardwareMap hardwareMap, String thing, double kp,double ki, double kd) {
        motor = hardwareMap.get(DcMotorEx.class, thing);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer = new ElapsedTime();
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
    public void ResetMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
    public boolean moveMotorWithDeg(double velocity,int deg) {
        if(!Reset) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Reset = true;
        }
        moveWithPower(velocity);
        if(Math.abs(motor.getCurrentPosition()) > deg) {
            moveWithPower(0);
            Reset = false;
            return true;
        }
        return false;
    }
    //ref = where we would like to be
    //state = currently are
    private double PIDControl(double reference, double state) {
        double error = reference - state;
        //telemetry.addData("state",state);
        //if(reference < state) return 0;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public boolean MoveWithPID(double reference) {
        double output = PIDControl(reference,motor.getCurrentPosition());
        moveWithPower(output);
        return output < 0.05;
    }

    public void StopMotor() {
        motor.setPower(0);
    }
    public boolean IsMotorBusy() {
        return motor.isBusy();
    }
    public void moveWithPower(double velocity) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(velocity);
    }
    public double getPower() {
        return motor.getPower();
    }
    public void SetMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }
    public boolean runToPosition(int currentPosition, int position, double power) {
        motor.setTargetPosition(position);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return Math.abs(currentPosition - position) <= 1;
    }
    public void setDirection(DcMotorSimple.Direction d) {
        motor.setDirection(d);
    }
    public void EnableMotor() {
        motor.setMotorEnable();
    }
    public void DisableMotor() {
        motor.setMotorDisable();
    }
    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }
    public void setVelocity(double angularRate, AngleUnit angle) {
        motor.setVelocity(angularRate, angle);
    }
    public int getPosition() {
        return motor.getCurrentPosition();
    }
}
