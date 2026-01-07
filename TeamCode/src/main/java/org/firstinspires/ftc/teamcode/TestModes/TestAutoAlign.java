package org.firstinspires.ftc.teamcode.TestModes;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;

@TeleOp
public class TestAutoAlign extends LinearOpMode{
    @Override
    public void runOpMode() throws  InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Camera camera = new Camera(hardwareMap);
        boolean x = false;
        waitForStart();
        double[] target = new double[]{ -25,11,40};
        while(opModeIsActive()){
            double[] data = camera.telemetryAprilTag(telemetry);
            telemetry.addData("a",data[0]);
            if(gamepad1.a && !x){
                x = true;
            }

//            if(x){
//                x = !drive.AlignToTag(data, target,telemetry).run(new TelemetryPacket());
//            }
            telemetry.update();
        }
    }
}