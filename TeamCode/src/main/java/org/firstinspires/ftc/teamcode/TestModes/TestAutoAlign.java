package org.firstinspires.ftc.teamcode.TestModes;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;

@TeleOp
public class TestAutoAlign extends LinearOpMode{
    @Override
    public void runOpMode() throws  InterruptedException{
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        //Camera camera = new Camera(hardwareMap);
        boolean x = false;
        waitForStart();
        double[] target = new double[]{ -25,11,48};
        while(opModeIsActive()){
            double[] data = hardware.camera.telemetryAprilTag(telemetry);
            telemetry.addData("id",data[0]);
            if(data.length > 1){
                telemetry.addData("x", data[1]);
                telemetry.addData("y", data[2]);
                telemetry.addData("Yaw", data[3]);
                telemetry.addData("distance", data[4]);
                telemetry.addData("yaw", data[5]);
            }

            if(gamepad1.a && !x){
                x = true;
            }

            if(x){
                x = !hardware.drive.AlignToTag(hardware, target,telemetry).run(new TelemetryPacket());
            }
            telemetry.update();
        }
    }
}