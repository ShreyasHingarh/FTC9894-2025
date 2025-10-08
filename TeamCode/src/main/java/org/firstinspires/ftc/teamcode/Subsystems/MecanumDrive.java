package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Enums.RealignStates;
import org.opencv.core.Point;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public final class MecanumDrive {
    public static final double SPEED_REDUCTION = 0.70;
    private static double power1;
    private static double power2;
    private static double power3;
    private static double power4;
    private static double CRAB = 1.15;
    public static boolean reset = false;
    public static double crabwalkCorrection = 0;
    public static double turn = 0;
    public static double adjustment = 0.018;

    public final DcMotorEx leftFront, leftRear, rightRear, rightFront, xEncoderMotor, yEncoderMotor;

    public final Encoder XEncoder;
    public final Encoder YEncoder;
    public final VoltageSensor voltageSensor;

    public final IMU imu;
    public Pose2d pose;

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        yEncoderMotor = hardwareMap.get(DcMotorEx.class, "parallelEncoder");
        xEncoderMotor = hardwareMap.get(DcMotorEx.class, "perpendicularEncoder");

        yEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        XEncoder = new Encoder() {
            private static final int REVERSAL = 1;

            @NonNull
            @Override
            public PositionVelocityPair getPositionAndVelocity() {
                // Get the raw values from the encoder port
                int rawPosition = xEncoderMotor.getCurrentPosition();
                double rawVelocity = xEncoderMotor.getVelocity();

                // Return the processed and raw values
                return new PositionVelocityPair(
                        rawPosition * REVERSAL,         // Processed position
                        (int) (rawVelocity * REVERSAL), // Processed velocity
                        rawPosition,                    // Raw position
                        (int) rawVelocity               // Raw velocity
                );
            }
            @NonNull
            @Override
            public DcMotorController getController() {
                return xEncoderMotor.getController();
            }
            @NonNull
            @Override
            public DcMotorSimple.Direction getDirection() {
                return xEncoderMotor.getDirection();
            }
            @Override
            public void setDirection(@NonNull DcMotorSimple.Direction direction) {
                xEncoderMotor.setDirection(direction);
            }
        };
        YEncoder = new Encoder() {
            private static final int REVERSAL = 1;

            @NonNull
            @Override
            public PositionVelocityPair getPositionAndVelocity() {
                // Get the raw values from the encoder port
                int rawPosition = yEncoderMotor.getCurrentPosition();
                double rawVelocity = yEncoderMotor.getVelocity();

                // Return the processed and raw values
                return new PositionVelocityPair(
                        rawPosition * REVERSAL,         // Processed position
                        (int) (rawVelocity * REVERSAL), // Processed velocity
                        rawPosition,                    // Raw position
                        (int) rawVelocity               // Raw velocity
                );
            }

            // The methods below are required by the interface but are less critical for dead wheels
            @NonNull
            @Override
            public DcMotorController getController() {
                return yEncoderMotor.getController();
            }

            @NonNull
            @Override
            public DcMotorSimple.Direction getDirection() {
                return yEncoderMotor.getDirection();
            }

            @Override
            public void setDirection(@NonNull DcMotorSimple.Direction direction) {
                yEncoderMotor.setDirection(direction);
            }
        };

        LazyImu lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu = lazyImu.get();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public Action moveFromPointToPoint(){

    }
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftRear.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightRear.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public Pose2d getPosition() {
        return pose;
    }

    public void setPosition(double x, double y, double theta){
        pose = new Pose2d(x,y,theta);
    }

    public void resetPosition(){
        pose = new Pose2d(0,0, 0);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveChassisWithPower(double powerOne, double powerTwo, double powerThree, double powerFour) {
        leftFront.setPower(powerOne);
        rightFront.setPower(powerTwo);
        leftRear.setPower(powerThree);
        rightRear.setPower(powerFour);
    }

    public void SetTargetPosition(int target) {
        leftRear.setTargetPosition(target);
        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightRear.setTargetPosition(target);
    }
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void resetIfNecessary() {
        if (getRawExternalHeading() < -1 || getRawExternalHeading() > 1) {
            imu.resetYaw();
        }
    }
    public void driveWithInput(double x, double y, double triggers, Telemetry telemetry) {
        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(Drivetrain.turn), 1);
        double correctX = x * 1.1; // check this
        power1 = (y + correctX + turn) * SPEED_REDUCTION * CRAB;
        power2 = (y - correctX - turn) * SPEED_REDUCTION * CRAB;
        power3 = (y - correctX + turn) * SPEED_REDUCTION;
        power4 = (y + correctX - turn) * SPEED_REDUCTION;

        //turn
        if (triggers != 0) {
            turn = triggers;
            reset = true;
        } else {
            if (reset) {
                resetIfNecessary();
                reset = false;
            }
        }

        if (x > 0) {
            //strafing right
            //+
            //-
            //-
            //+
            //Correction
            //-
            //-
            //+
            //+
            power1 -= crabwalkCorrection;
            power2 += crabwalkCorrection;
            power3 -= crabwalkCorrection;
            power4 += crabwalkCorrection;
        } else if (x < 0) {
            //strafing left
            //-
            //+
            //+
            //-
            power1 += crabwalkCorrection;
            power2 -= crabwalkCorrection;
            power3 += crabwalkCorrection;
            power4 -= crabwalkCorrection;
        }
        telemetry.addData("power1", power1);
        telemetry.addData("power2", power2);
        telemetry.addData("power3", power3);
        telemetry.addData("power4", power4);
        power1 = Range.clip(power1, -1, 1);
        power2 = Range.clip(power2, -1, 1);
        power3 = Range.clip(power3, -1, 1);
        power4 = Range.clip(power4, -1, 1);

        leftFront.setPower(power1);
        rightFront.setPower(power2);
        leftRear.setPower(power3);
        rightRear.setPower(power4);

        //reset encoders when robot stops
        if (x == 0 && y == 0 && triggers == 0) {
            resetIfNecessary();
            turn = 0;
            crabwalkCorrection = 0;
        }
    }
    private static boolean ColorCheck(ColorSensor sensor, int a) {
        return sensor.red() > a && sensor.green() > a;
    }


    private double InchesToTicks(int inches){
        return inches * (336/(Math.PI * 2.95276));
    }
}
