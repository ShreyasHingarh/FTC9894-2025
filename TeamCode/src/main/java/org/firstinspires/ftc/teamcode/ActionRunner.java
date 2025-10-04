package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ActionRunner {
    int index = 0;
    Action[] EveryAction;
    Telemetry tele;
    public ActionRunner(Telemetry telemetry, Action... list){
        EveryAction = list;
        tele = telemetry;
    }

    public boolean Run(TelemetryPacket pack){
        if(EveryAction[index].run(pack)){
            index++;
            if(index == EveryAction.length) {
                return true;
            }
        }
        return false;
    }
}
