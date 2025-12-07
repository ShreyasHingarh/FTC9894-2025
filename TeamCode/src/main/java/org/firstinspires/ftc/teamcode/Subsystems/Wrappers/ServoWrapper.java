package org.firstinspires.ftc.teamcode.Subsystems.Wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoWrapper {
    private Servo servoImplEx;
    private ElapsedTime timer;
    private boolean Reset = false;

    public ServoWrapper(HardwareMap hardwareMap, String thing) {
        servoImplEx = hardwareMap.get(Servo.class,thing);
    }
    public void SetDirection(Servo.Direction direction) {
        servoImplEx.setDirection(direction);
    }
    public void setPosition(double pos) {
        servoImplEx.setPosition(pos);
    }
    public boolean setPosition(double position, int time) {
        if(!Reset) {
            timer.reset();
            Reset = true;
        }
        servoImplEx.setPosition(position);
        if(timer.milliseconds() > time)
        {
            Reset = false;
            return true;
        }
        return false;
    }
    public double getPosition() {
        return servoImplEx.getPosition();
    }
}
