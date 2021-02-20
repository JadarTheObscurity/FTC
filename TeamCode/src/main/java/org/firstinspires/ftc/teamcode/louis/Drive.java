package org.firstinspires.ftc.teamcode.louis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Louis Drive", group = "Louis")
public class Drive extends OpMode {
    Louis louis;
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void init() {
        louis = new Louis(hardwareMap);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        louis.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(gamepad1.a) louis.reset_pos();
        for(int i = 0; i < 4; i++) {
            packet.put("curr" + i + " ", louis.driveTrain.motors.get(i).getCurrentPosition());
            packet.put("last " + i + " ", louis.driveTrain.last_pos[i]);
        }
        packet.put("x", louis.driveTrain.get_x());
        packet.put("y", louis.driveTrain.get_y());
        packet.put("imu", louis.driveTrain.get_heading());
        dashboard.sendTelemetryPacket(packet);
    }
}
