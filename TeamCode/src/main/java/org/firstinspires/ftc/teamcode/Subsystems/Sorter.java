package org.firstinspires.ftc.teamcode.Subsystems;
import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Enums.AutoLaunch;
import org.firstinspires.ftc.teamcode.Enums.BallColor;
import org.firstinspires.ftc.teamcode.Enums.LaunchStates;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.ServoWrapper;

public class Sorter {
    public DcMotorWrapper sortMotor;
    public ServoWrapper servo;
    public ColorSensor color1;
    public ColorSensor color2;
    public ColorSensor color3;
    public LaunchStates launchState;
    public AutoLaunch autoLaunch;
    public BallColor[] holder = new BallColor[]{
        BallColor.None,
        BallColor.None,
        BallColor.None
    };
    private final double SERVOLAUNCH = 0;
    private final double SERVOPOSITION = 0.18;
    private final double SERVOTIME = 600;
    private final double SortSpeed = 0.2;

    private final ElapsedTime timer;
    public int sortPosition = 0;
    public int currentPosition = 0;
    public int currentDegrees = 0;
    public boolean isFull = false;
    public int indexToSpinTo = -1;
    public int[] orderToLaunch = new int[] {-1,-1,-1};

    public Sorter(HardwareMap hardwareMap){
        sortMotor = new DcMotorWrapper(hardwareMap, "sorter", 0,0,0);
        sortMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sortMotor.runToPosition(sortMotor.getPosition(),0, 0.1);
        servo = new ServoWrapper(hardwareMap,"kickerReal");
        servo.setPosition(SERVOPOSITION);
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color1.enableLed(true);
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color2.enableLed(true);
        color3 = hardwareMap.get(ColorSensor.class, "color3");
        color3.enableLed(true);
        launchState = LaunchStates.MoveSort;
        autoLaunch = AutoLaunch.reset;
        timer = new ElapsedTime();
    }
    public int[] getSensorValue(ColorSensor color){
        return new int[]{color.red(), color.green(), color.blue(), (int) JavaUtil.colorToHue((Color.rgb(color.red(), color.green(),color.blue())))};
    }
    public Action spinSorterToIntake(int nextPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(nextPosition == -1) return true;

                currentDegrees = returnRangedDegrees();
                if(nextPosition == 0){
                    if(sortMotor.runToPosition(currentDegrees,0, SortSpeed)){
                        currentPosition = nextPosition;
                        sortPosition = 0;
                        return true;
                    }
                } else if(nextPosition == 1){
                    if(sortMotor.runToPosition(currentDegrees,180, SortSpeed)){
                        currentPosition = nextPosition;
                        sortPosition = 180;
                        return true;
                    }
                } else if(nextPosition == 2){
                    if(sortMotor.runToPosition(currentDegrees,360,  SortSpeed)){
                        currentPosition = nextPosition;
                        sortPosition = 360;
                        return true;
                    }
                }
                return false;
            }
        };
    }
    public int returnRangedDegrees(){
        int i = sortMotor.getPosition();
        while(i >= 540){
            i -= 540;
        }
        while(i < 0){
            i += 540;
        }
        return i;
    }
    public Action spinSorterToLaunch(int nextPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (nextPosition == -1) return true;

                currentDegrees = returnRangedDegrees();
                if(nextPosition == 0){
                    if(sortMotor.runToPosition(currentDegrees,270, SortSpeed)){
                        currentPosition = nextPosition;
                        sortPosition = 270;
                        return true;
                    }
                } else if(nextPosition == 1){
                    if(sortMotor.runToPosition(currentDegrees,450, SortSpeed)){
                        currentPosition = nextPosition;
                        sortPosition = 450;
                        return true;
                    }
                } else if(nextPosition == 2){
                    if(sortMotor.runToPosition(currentDegrees,90,SortSpeed)){
                        currentPosition = nextPosition;
                        sortPosition = 90;
                        return true;
                    }
                }
                return false;
            }
        };
    }

    public Action reset(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servo.setPosition(SERVOPOSITION);
                launchState = LaunchStates.MoveSort;
                autoLaunch = AutoLaunch.reset;
                int[] a = getSensorValue(color3);
                if(a[0] < 800 && a[1] < 800 && a[2] < 800){
                    sortMotor.moveWithPower(0);
                    sortMotor.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return true;
                }
                return false;
            }
        };
    }
    private int[] getEmptySpots(){
        int[] positions = new int[] {-1,-1,-1};
        int i = 0;
        int sum = 0;
        for(int index = 0; index < 3;index++){
            if(holder[index] == BallColor.None){
                positions[i] = index;
                i++;
            } else{
                sum -= 1;
            }
        }
        isFull = sum == -3;
        return positions;
    }
    public BallColor sensorSeesBall(ColorSensor color){
        int[] a = getSensorValue(color);
        if(a[3] > 125 && a[3] < 150 && a[1] > 50){
            return BallColor.Green;
        } else if((a[3] < 90 && a[0] > 120 && a[2] > 120)
                        || (a[3] > 230)) {
            return BallColor.Purple;
        }
        return BallColor.None;
    }

    public void organizeSorter(TelemetryPacket pack) {
        if(indexToSpinTo != -1) {
            if(spinSorterToIntake(indexToSpinTo).run(pack)){
                indexToSpinTo = -1;
            }
            return;
        }
        BallColor sensor1Value = sensorSeesBall(color1);
        BallColor sensor2Value = sensorSeesBall(color2);
        if (!isFull && ((sensor1Value == BallColor.Green && sensor2Value == BallColor.Green)
                || (sensor1Value == BallColor.None && sensor2Value == BallColor.Green)
                || (sensor1Value == BallColor.Green && sensor2Value == BallColor.None)
                || (sensor1Value == BallColor.Purple && sensor2Value == BallColor.Purple)
                || (sensor1Value == BallColor.Purple && sensor2Value == BallColor.None)
                || (sensor1Value == BallColor.None && sensor2Value == BallColor.Purple))) {
            holder[currentPosition] = sensor1Value != BallColor.None ? sensor1Value : sensor2Value;
            int[] emptySpots = getEmptySpots();
            if (isFull) {
                return;
            }
            indexToSpinTo = emptySpots[0];
        }
    }

    public int[] GetOrder(Hardware hardware){
        BallColor[] requiredOrder = hardware.camera.Order;
        int[] orderToSpin = new int[3];
        int numberOfPurple = 0;
        int numberOfGreen = 0;
        int positionOfGreen = -1;
        for(int i = 0;i < orderToSpin.length;i++){
            if(holder[i] == BallColor.Green) {
                numberOfGreen++;
                positionOfGreen = i;
            }else if(holder[i] == BallColor.Purple){
                numberOfPurple++;
            }
        }
        if(numberOfGreen == 1 && numberOfPurple == 2){
            if(requiredOrder[0] == BallColor.Green){
                if(positionOfGreen == 0){
                    orderToSpin[0] = 0;
                    orderToSpin[1] = 2;
                    orderToSpin[2] = 1;
                }
                else{
                    orderToSpin[0] = positionOfGreen;
                    orderToSpin[1] = Math.abs(positionOfGreen - 1) % 3;
                    orderToSpin[2] = Math.abs(positionOfGreen + 1) % 3;
                }
            }else if(requiredOrder[1] == BallColor.Green){
                if(positionOfGreen == 0){
                    orderToSpin[0] = 2;
                    orderToSpin[1] = 0;
                    orderToSpin[2] = 1;
                }
                else{
                    orderToSpin[0] = Math.abs(positionOfGreen - 1) % 3;
                    orderToSpin[1] = positionOfGreen;
                    orderToSpin[2] = Math.abs(positionOfGreen + 1) % 3;
                }
            } else{
                if(positionOfGreen == 0){
                    orderToSpin[0] = 1;
                    orderToSpin[1] = 2;
                    orderToSpin[2] = 0;
                }
                else{
                    orderToSpin[0] = Math.abs(positionOfGreen + 1) % 3;
                    orderToSpin[1] = Math.abs(positionOfGreen - 1) % 3;
                    orderToSpin[2] = positionOfGreen;
                }

            }
            return orderToSpin;
        }

        for(int i = 0;i < 3;i++){
            if(holder[i] != BallColor.None){
                orderToSpin[i] = i;
            } else{
                orderToSpin[i] = -1;
            }
        }
        return orderToSpin;
    }
    public Action launchAtPosition(int position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch(launchState){
                    case MoveSort:
                        if(spinSorterToLaunch(position).run(telemetryPacket)){
                            launchState = LaunchStates.wait;
                            timer.reset();
                        }
                        break;
                    case wait:
                        if(timer.milliseconds() > 400){
                            launchState = LaunchStates.servoLaunch;
                            timer.reset();
                        }
                        break;
                    case servoLaunch:
                        servo.setPosition(SERVOLAUNCH);
                        if(timer.milliseconds() > SERVOTIME){
                            launchState = LaunchStates.servoReset;
                            timer.reset();
                        }
                        break;
                    case servoReset:
                        servo.setPosition(SERVOPOSITION);
                        if(timer.milliseconds() > SERVOTIME){
                            launchState = LaunchStates.MoveSort;
                            holder[position] = BallColor.None;
                            isFull = false;
                            return true;
                        }
                        break;
                }
                return false;
            }
        };
    }
    public Action launch(Hardware hardware){
        return new Action (){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch(autoLaunch){
                    case reset:
                        orderToLaunch = GetOrder(hardware);
                        autoLaunch = AutoLaunch.launch1;
                        break;
                    case launch1:
                        if(orderToLaunch[0] == -1 || launchAtPosition(orderToLaunch[0]).run(telemetryPacket)){
                            autoLaunch = AutoLaunch.launch2;
                        }
                        break;
                    case launch2:
                        if(orderToLaunch[1] == -1 || launchAtPosition(orderToLaunch[1]).run(telemetryPacket)){
                            autoLaunch = AutoLaunch.launch3;
                        }
                        break;
                    case launch3:
                        if(orderToLaunch[2] == -1 || launchAtPosition(orderToLaunch[2]).run(telemetryPacket)){
                            autoLaunch = AutoLaunch.resetSorter;
                        }
                        break;
                    case resetSorter:
                        if(spinSorterToIntake(0).run(telemetryPacket)){
                            autoLaunch = AutoLaunch.reset;
                            return true;
                        }
                        break;
                }
                return false;
            }
        };
    }


}
