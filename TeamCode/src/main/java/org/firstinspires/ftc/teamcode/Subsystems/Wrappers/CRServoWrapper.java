package org.firstinspires.ftc.teamcode.Subsystems.Wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CRServoWrapper {
    private CRServo crServo;
    private ElapsedTime timer;
    private boolean hasReset = false;
    public CRServoWrapper(HardwareMap hardwareMap, String thing) {
        crServo = hardwareMap.get(CRServo.class,thing);
    }
    public void SetPower(double power) {
        crServo.setPower(power);
    }
    public void SetDirection(DcMotorSimple.Direction direction) {
        crServo.setDirection(direction);
    }
    public boolean SetPowerForTime(double power, int time) {
        if(!hasReset) {
            timer.reset();
            hasReset = true;
        }
        SetPower(power);
        if(timer.milliseconds() > time) {
            SetPower(0);
            hasReset = false;
            return true;
        }
        return false;
    }
}
