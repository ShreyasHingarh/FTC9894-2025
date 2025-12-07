package org.firstinspires.ftc.teamcode.Subsystems;
import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Enums.AutoLaunch;
import org.firstinspires.ftc.teamcode.Enums.BallColor;
import org.firstinspires.ftc.teamcode.Enums.LaunchStates;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.ServoWrapper;

public class Sorter {
    public DcMotorWrapper sortMotor;
    private ServoWrapper servo;
    public ColorSensor color1;
    public ColorSensor color2;
    public LaunchStates launchState;
    public AutoLaunch autoLaunch;
    public BallColor[] holder = new BallColor[]{
        BallColor.None,
        BallColor.None,
        BallColor.None
    };
    private final double SERVOLAUNCH = 0.43;
    private final double SERVOPOSITION = 0.5;
    private final double SERVOTIME = 750;
    private final ElapsedTime timer;
    private final int distanceBetweenTwo = 180;
    private int sortPosition = 0;
    private final double SortSpeed = 0.3;
    public int currentPosition = 0;
    public boolean isFull = false;
    public int indexToSpinTo = -1;
    public int[] orderToLaunch = new int[] {-1,-1,-1};

    public Sorter(HardwareMap hardwareMap){
        sortMotor = new DcMotorWrapper(hardwareMap, "sorter", 0,0,0);
        sortMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = new ServoWrapper(hardwareMap,"kickerReal");
        servo.setPosition(SERVOPOSITION);
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color1.enableLed(true);
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color2.enableLed(true);
        launchState = LaunchStates.MoveSort;
        autoLaunch = AutoLaunch.moveOffset;
        timer = new ElapsedTime();
    }
    public int[] getSensorValue(ColorSensor color){
        return new int[]{color.red(), color.green(), color.blue(), (int) JavaUtil.colorToHue((Color.rgb(color.red(), color.green(),color.blue())))};
    }
    private boolean MoveUp(int nextPosition){
        return (currentPosition == 0 && nextPosition == 1)
                || (currentPosition == 1 && nextPosition == 2)
                || (currentPosition == 2 && nextPosition == 0);
    }
    private boolean MoveBack(int nextPosition){
        return (currentPosition == 0 && nextPosition == 2)
                || (currentPosition == 1 && nextPosition == 0)
                || (currentPosition == 2 && nextPosition == 1);
    }
    public Action spinSorterToIntake(int nextPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(nextPosition == -1) return true;
                if(MoveUp(nextPosition)){
                    sortMotor.runToPosition(nextPosition * distanceBetweenTwo ,SortSpeed);
                    if(((nextPosition == 1 || nextPosition == 2) && Math.abs(sortMotor.getPosition() - nextPosition * distanceBetweenTwo) < 3)
                            || (nextPosition == 0 && (sortMotor.getPosition() > 538 || sortMotor.getPosition() < 3))){
                        currentPosition = nextPosition;
                        sortPosition = sortMotor.getPosition();
                        return true;
                    }
                    return false;
                } else if(MoveBack(nextPosition)) {
                    sortMotor.runToPosition(nextPosition * distanceBetweenTwo ,-SortSpeed);
                    if(((nextPosition == 1 || nextPosition == 0) && Math.abs(sortMotor.getPosition() - (nextPosition * distanceBetweenTwo)) < 3)
                            || (nextPosition == 2 && Math.abs(sortMotor.getPosition() - (distanceBetweenTwo * 2)) < 3)) {
                        currentPosition = nextPosition;
                        sortPosition = sortMotor.getPosition();
                        return true;
                    }
                    return false;
                }
                return true;
            }
        };
    }

    public Action spinSorterToLaunch(int nextPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (nextPosition == -1) return true;
                if (MoveBack(nextPosition)) {
                    sortMotor.runToPosition(nextPosition == 2 ? 450 : nextPosition * distanceBetweenTwo + 90, -SortSpeed);
                    if (((nextPosition == 1 || nextPosition == 0) && Math.abs(sortMotor.getPosition() - (nextPosition * distanceBetweenTwo + 90)) < 3)
                            || (nextPosition == 2 && Math.abs(sortMotor.getPosition() - 450) < 3)) {
                        currentPosition = nextPosition;
                        sortPosition = sortMotor.getPosition();
                        return true;
                    }
                    return false;
                } else if(MoveUp(nextPosition)) {
                    sortMotor.runToPosition(nextPosition * distanceBetweenTwo + 90,SortSpeed);
                    if(((nextPosition == 1 || nextPosition == 2) && Math.abs(sortMotor.getPosition() - (nextPosition * distanceBetweenTwo + 90)) < 3)
                            || (nextPosition == 0 && Math.abs(sortMotor.getPosition() - 90) < 3) ) {
                        currentPosition = nextPosition;
                        sortPosition = sortMotor.getPosition();
                        return true;
                    }
                    return false;
                }
                return true;
            }
        };
    }

    public Action reset(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servo.setPosition(SERVOPOSITION);
                launchState = LaunchStates.MoveSort;
                autoLaunch = AutoLaunch.launch1;
                sortMotor.runToPosition(0,SortSpeed);
                return sortMotor.getPosition() == 0;
            }
        };
    }
    public int[] getEmptySpots(){
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
        if(a[3] > 125 && a[3] < 150 && a[1] > 60){
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
        if (!isFull && (sensor1Value == BallColor.Green && sensor2Value == BallColor.Green)
                || (sensor1Value == BallColor.Purple && sensor2Value == BallColor.Purple)
                || (sensor1Value == BallColor.Purple && sensor2Value == BallColor.None)
                || (sensor1Value == BallColor.None && sensor2Value == BallColor.Purple)) {
            holder[currentPosition] = sensor1Value != BallColor.None ? sensor1Value : sensor2Value;
            int[] emptySpots = getEmptySpots();
            if (isFull) {
                return;
            }
            indexToSpinTo = emptySpots[0];
        }
    }
    public Action moveOffSet(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sortMotor.runToPosition(sortPosition + 90, SortSpeed);
                if(sortMotor.getPosition() == sortPosition + 90){
                    sortPosition = sortMotor.getPosition();
                    return true;
                }
                return false;
            }
        };
    }
    public Action resetOffset(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sortMotor.runToPosition(sortPosition - 90, SortSpeed);
                if( sortMotor.getPosition() == sortPosition - 90){
                    sortPosition = sortMotor.getPosition();
                    return true;
                }
                return false;
            }
        };
    }
    public int[] GetOrder(BallColor[] requiredOrder){
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
                orderToSpin[0] = positionOfGreen;
                orderToSpin[1] = Math.abs(positionOfGreen - 1) % 3;
                orderToSpin[2] = Math.abs(positionOfGreen + 1) % 3;
            }else if(requiredOrder[1] == BallColor.Green){
                orderToSpin[0] = Math.abs(positionOfGreen - 1) % 3;
                orderToSpin[1] = positionOfGreen;
                orderToSpin[2] = Math.abs(positionOfGreen + 1) % 3;
            } else{
                orderToSpin[0] = Math.abs(positionOfGreen - 1) % 3;
                orderToSpin[1] = Math.abs(positionOfGreen + 1) % 3;
                orderToSpin[2] = positionOfGreen;
            }
            return orderToSpin;
        }

        int[] indexes = new int[]{currentPosition, Math.abs(currentPosition - 1) % 3, Math.abs(currentPosition + 1) % 3};
        for(int i = 0;i < indexes.length;i++){
            if(holder[indexes[i]] != BallColor.None){
                orderToSpin[i] = indexes[i];
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
    public Action launch(BallColor[] order){
        return new Action (){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch(autoLaunch){
                    case moveOffset:
                            orderToLaunch = GetOrder(order);
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
//                    case resetOffset:
//                        if(resetOffset().run(telemetryPacket)){
//                            autoLaunch = AutoLaunch.resetSorter;
//                        }
//                        break;
                    case resetSorter:
                        if(spinSorterToIntake(0).run(telemetryPacket)){
                            autoLaunch = AutoLaunch.moveOffset;
                        }
                        return true;
                }
                return false;
            }
        };
    }

//    public Action launchAtColor(BallColor color) {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                int position = getPositionOfColor(color);
//                pos = position;
//                if(position == -1) return true;
//                switch(launchState){
//                    case MoveSort:
//                        if(spinSorterToLaunch((position + 1) % 3).run(telemetryPacket)){
//                            launchState = LaunchStates.servoLaunch;
//                            timer.reset();
//                        }
//                        break;
//                    case servoLaunch:
//                        servo.setPosition(0.05);
//                        if(timer.milliseconds() > 400){
//                            launchState = LaunchStates.servoReset;
//                            timer.reset();
//                        }
//                        break;
//                    case servoReset:
//                        servo.setPosition(0.65);
//                        if(timer.milliseconds() > 400){
//                            launchState = LaunchStates.MoveSort;
//                            holder[position] = BallColor.None;
//                            isFull = false;
//                            return true;
//                        }
//                        break;
//                }
//                return false;
//            }
//        };
//    }
}
