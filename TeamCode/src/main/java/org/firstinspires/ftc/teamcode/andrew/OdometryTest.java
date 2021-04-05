package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.WayPoint;

@TeleOp(group = "Andrew")
//@Disabled
public class OdometryTest extends LinearOpMode {
    Andrew andrew;

    @Override
    public void runOpMode() throws InterruptedException {
        andrew = new Andrew(hardwareMap);
        andrew.setStartPose(new Pose2d(0, -150, 0));
        waitForStart();
        test_odometry();
        andrew.finish();
    }

    void test_odometry(){
        moveTo(new WayPoint(0, 150, Math.toRadians(180)));
        moveTo(new WayPoint(0, -150, Math.toRadians(0)));
    }

    void moveTo(WayPoint target){
        andrew.reset_all();
        while(opModeIsActive() && !andrew.moveTo(target)){
            andrew.update();
        }
        andrew.stop_all();
    }

}
