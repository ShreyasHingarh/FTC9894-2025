package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.Color;

@TeleOp
public class Teleop extends LinearOpMode {
    MainActions mainActions;
    TelemetryPacket pack;

    private boolean IsDrive(){
        return gamepad1.left_stick_x != 0 || -gamepad1.left_stick_y != 0 ||
                (gamepad1.right_trigger - gamepad1.left_trigger) != 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mainActions = new MainActions(hardwareMap,telemetry,Color.Red);
        pack = new TelemetryPacket();
        waitForStart();
        while(opModeIsActive()) {

            //DRIVE
            mainActions.hardware.drive.driveWithInput(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    (gamepad1.right_trigger - gamepad1.left_trigger), telemetry);
        }
    }
}
