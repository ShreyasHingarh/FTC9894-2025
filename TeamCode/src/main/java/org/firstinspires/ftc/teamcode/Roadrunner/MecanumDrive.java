package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
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
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Roadrunner.messages.*;
import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public static final double SPEED_REDUCTION = 1.0;
    public static boolean hasReset = false;
    private boolean isInitialized = false;
    private double targetHeading;

    // Tuning parameters (Adjust these based on your robot's weight/friction)
    // K_P: How aggressively to turn. If it oscillates, lower this.
    private static final double K_P = 1.5;
    // MIN_SPEED: Minimum power to ensure the robot actually moves at the end
    private static final double MIN_SPEED = 0.15;

    public static class Params {
        // IMU orientation

        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
        // Custom P-Controller Gains for AlignToTag

        public double P_LATERAL = 0.04; // For X error (strafe)
        public double P_AXIAL = 0.04;   // For Y error (forward/backward)
        public double P_HEADING = 0.04; // For heading error (turn)

        // Tolerance for AlignToTag
        public double LATERAL_TOLERANCE = 1; // in inches
        public double AXIAL_TOLERANCE = 1;   // in inches
        public double HEADING_TOLERANCE = 2;
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            imu = lazyImu.get();

//            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(
                    twist.line.value(),
                    headingDelta
            ));

            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new DriveLocalizer(pose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        
        
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
    public double getRawExternalHeading() {
        return localizer.getPose().heading.toDouble();
    }
    public void resetIfNecessary() {
        if (getRawExternalHeading() < -1 || getRawExternalHeading() > 1) {
            localizer.setPose(new Pose2d(localizer.getPose().position.x,localizer.getPose().position.y,0));
        }
    }
    public void driveWithInput(double x, double y, double turnInput, Telemetry telemetry) {
        double correctX = x * 1.1;
        double denominator = Math.max(Math.abs(y) + Math.abs(correctX) + Math.abs(turnInput), 1);
        double flPower = (y + correctX + turnInput) / denominator * SPEED_REDUCTION;
        double blPower = (y - correctX + turnInput) / denominator * SPEED_REDUCTION;
        double brPower = (y + correctX - turnInput) / denominator * SPEED_REDUCTION;
        double frPower = (y - correctX - turnInput) / denominator * SPEED_REDUCTION;

        // 5. Apply Powers
        leftFront.setPower(flPower);
        leftBack.setPower(blPower);
        rightBack.setPower(brPower);
        rightFront.setPower(frPower);

        telemetry.addData("Powers", "FL:%.2f BL:%.2f BR:%.2f FR:%.2f", flPower, blPower, brPower, frPower);
        telemetry.addData("Inputs", "X:%.2f Y:%.2f Turn:%.2f", x, y, turnInput);
    }
    public Action reset(Telemetry telemetry){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                MoveChassisWithPower(0,0,0,0);
                resetEncoders();
                return true;
            }
        };
    }
    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveChassisWithPower(double powerOne, double powerTwo, double powerThree, double powerFour) {
        leftFront.setPower(powerOne);
        rightFront.setPower(powerTwo);
        leftBack.setPower(powerThree);
        rightBack.setPower(powerFour);
    }
    public Action Move(int Inches, double speed,Telemetry telemetry) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double ticks = InchesToTicks(Inches);
                telemetry.addData("Ticks",ticks);
                if(!hasReset){
                    MoveChassisWithPower(0,0,0,0);
                    resetEncoders();
                    runWithEncoders();
                    hasReset = true;
                }
                MoveChassisWithPower(speed,speed,speed,speed);
                double avg = (double) (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition())
                        + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition())) / 4;
                telemetry.addData("avg", avg);
                telemetry.update();
                if(avg > ticks){
                    MoveChassisWithPower(0,0,0,0);
                    resetEncoders();
                    runWithEncoders();
                    hasReset = false;
                    return true;
                }
                return false;
            }
        };
    }
    private boolean initialized = false;
    private double actionTargetHeading = 0;

    /// positive degrees is turning right, negative is turning left
    /// positive degrees is turning LEFT (Counter-Clockwise) by standard math.
    /// If you want positive to turn RIGHT, change: Math.toRadians(-degrees)
    public Action Turn(int degrees, double maxSpeed, Telemetry telemetry) {
        return new Action() {
            // FIX 1: Move these variables HERE so they are unique to *this* specific turn
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Initialize the target once per Action
                if (!initialized) {
                    localizer.update(); // Get fresh start pose
                    double currentHeading = localizer.getPose().heading.toDouble();

                    // Standard Coordinate System: Positive = Left (CCW).
                    actionTargetHeading = currentHeading + Math.toRadians(degrees);
                    initialized = true;
                }

                localizer.update(); // Update position
                double currentHeading = localizer.getPose().heading.toDouble();
                telemetry.addData("pos", localizer.getPose().heading.toDouble());
                // 1. Calculate Error
                double error = actionTargetHeading - currentHeading;

                // 2. Angle Wrapping
                while (error > Math.PI) error -= 2 * Math.PI;
                while (error <= -Math.PI) error += 2 * Math.PI;
                telemetry.addData("err", error);

                // 3. Check for completion
                if (Math.abs(error) < Math.PI/12) {
                    MoveChassisWithPower(0, 0, 0, 0);
                    initialized = false;
                    actionTargetHeading = 0;
                    return true;
                }

                // 4. Calculate Proportional Power (P-Controller)
                double desiredPower = error * K_P;

                // 5. Clip power
                double turnPower = Math.copySign(
                        Math.min(Math.abs(desiredPower), Math.abs(maxSpeed)),
                        desiredPower
                );

                // Min speed boost
                if (Math.abs(turnPower) < MIN_SPEED) {
                    turnPower = Math.copySign(MIN_SPEED, turnPower);
                }

                // Apply Power (Standard Mecanum: Left -, Right + for Left Turn)
                MoveChassisWithPower(-turnPower, turnPower, -turnPower, turnPower);

                // Driver Station Telemetry
                telemetry.addData("Target (rad)", actionTargetHeading);
                telemetry.addData("Current (rad)", currentHeading);
                telemetry.addData("Error (deg)", Math.toDegrees(error));
                telemetry.update();

                // FTC Dashboard Telemetry (Graphing)
                telemetryPacket.put("Turn Error", Math.toDegrees(error));
                telemetryPacket.put("Turn Power", turnPower);
                telemetryPacket.put("Heading", Math.toDegrees(currentHeading));
                telemetryPacket.put("Target", Math.toDegrees(actionTargetHeading));

                return false;
            }
        };
    }
    private double ticks;
    private double strafePower;
    public Action Strafe (int Inches, double speed, Telemetry telemetry){
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("Target Ticks", ticks);

                if(!hasReset){
                    MoveChassisWithPower(0,0,0,0);
                    resetEncoders();
                    runWithEncoders();
                    ticks = InchesToTicks(Math.abs(Inches));
                    strafePower = Math.copySign(speed, Inches);
                    hasReset = true;
                }
                MoveChassisWithPower(strafePower, -strafePower, -strafePower, strafePower);

                double avg = (double) (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition())
                        + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition())) / 4;

                if(avg > ticks){
                    MoveChassisWithPower(0,0,0,0);
                    resetEncoders();
                    runWithEncoders();
                    hasReset = false;
                    return true;
                }

                telemetry.addData("Current Avg Ticks", avg);
                telemetry.addData("Strafe Power", strafePower);
                telemetry.update();
                return false;
            }
        };
    }
    public static double norm(double angle) {
        double adjusted = angle % (2 * Math.PI);
        if (adjusted > Math.PI) {
            adjusted -= 2 * Math.PI;
        }
        if (adjusted <= -Math.PI) {
            adjusted += 2 * Math.PI;
        }
        return adjusted;
    }
    public Action AlignToTag(Hardware hardware, double[] target, Telemetry telemetry){
        return new Action() {
            private final double targetX = target[0];
            private final double targetY = target[1];
            private final double targetHeading = target[2];

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double[] positions = hardware.camera.telemetryAprilTag(telemetry);
                // Stop motion if tag data is missing or incomplete
                if (positions == null || positions.length < 3) {
                    MoveChassisWithPower(0, 0, 0, 0);
                    telemetry.addData("Alignment Status", "Missing Tag Data");
                    return false;
                }

                // Calculate errors relative to the target
                double errorX = positions[1] - targetX; // Lateral (strafe) error
                double errorY = positions[2] - targetY; // Axial (forward/backward) error
                double errorHeading = norm(positions[3] - targetHeading); // Angular (turn) error
                telemetry.addData("errorx", errorX);
                telemetry.addData("errory", errorY);
                telemetry.addData("errorHeading", errorHeading);
                // --- P-Controller Calculations ---
                double maxPower = 0.35;
                // Power = -Error * Gain (The negative sign ensures the robot drives *towards* the target)
                double lateralPower = com.acmerobotics.roadrunner.Math.clamp(-errorX * PARAMS.P_LATERAL, -maxPower, maxPower);
                double axialPower = com.acmerobotics.roadrunner.Math.clamp(-errorY * PARAMS.P_AXIAL, -maxPower, maxPower);
                double turnPower = com.acmerobotics.roadrunner.Math.clamp(-errorHeading * PARAMS.P_HEADING, -maxPower, maxPower);

                // --- Movement Check ---
                boolean isXAligned = Math.abs(errorX) < PARAMS.LATERAL_TOLERANCE;
                boolean isYAligned = Math.abs(errorY) < PARAMS.AXIAL_TOLERANCE;
                boolean isHeadingAligned = Math.abs(errorHeading) < PARAMS.HEADING_TOLERANCE;

                if (isXAligned && isYAligned && isHeadingAligned) {
                    // Robot is within tolerance
                    MoveChassisWithPower(0, 0, 0, 0);
                    return true; // Action is complete
                }
                driveWithInput(lateralPower, axialPower, -turnPower, telemetry);

                // --- Telemetry for Debugging ---
                telemetry.addData("Target Alignment", "X:%.2f, Y:%.2f, H:%.2f", targetX, targetY, Math.toDegrees(targetHeading));
                telemetry.addData("Error (Inches, Deg)", "X:%.2f, Y:%.2f, H:%.2f", errorX, errorY, Math.toDegrees(errorHeading));
                telemetry.addData("Powers", "X:%.2f, Y:%.2f, H:%.2f", lateralPower, axialPower, turnPower);
                telemetry.addData("Alignment Status", "X:%b, Y:%b, H:%b", isXAligned, isYAligned, isHeadingAligned);
                telemetry.update();
                return false; // Action is still running
            }
        };
    }

    private double InchesToTicks(int inches){
        return inches * (336/(Math.PI * 2.95276));
    }
}
