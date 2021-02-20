package org.firstinspires.ftc.teamcode.louis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Louis Auto 1", group = "Louis")
public class Auto1 extends OpMode {
    Louis louis;
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void init() {
        louis = new Louis(hardwareMap);
        dashboard = FtcDashboard.getInstance();

    }

    int curr_stage = 0;

    enum Stage{
        Start,
        move1,
        move2,
//        move3,
        End
    }

    @Override
    public void loop() {
//        louis.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        packet.put("x", louis.driveTrain.get_x());
        packet.put("y", louis.driveTrain.get_y());
        packet.put("imu", louis.driveTrain.get_heading());
        packet.put("target x", louis.control.x_target);
        dashboard.sendTelemetryPacket(packet);

        if(curr_stage == Stage.Start.ordinal()){
            nextStage();
        }
        if(curr_stage == Stage.move1.ordinal()){
            if(louis.moveTo(-120, 0, 0)) nextStage();
        }
        else if(curr_stage == Stage.move2.ordinal()){
            if(louis.moveTo(120, 0, 0)) setStage(Stage.move1.ordinal());
        }
//        else if (curr_stage == Stage.move3.ordinal()){
//            if(louis.moveTo(-50, 0, 0)) nextStage();
//        }
        else if(curr_stage == Stage.End.ordinal()){
            louis.move(0, 0, 0);
        }
    }


    void nextStage(){
        louis.reset_pos();
        curr_stage++;
    }

    void setStage(int num){
        nextStage();
        curr_stage = num;
    }
}
