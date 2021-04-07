package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.WayPoint;

@Autonomous(group = "Andrew")
public class AndrewInterferBlue extends LinearOpMode {
    Andrew andrew;
    @Override
    public void runOpMode() throws InterruptedException {
        andrew = new Andrew(hardwareMap);
        andrew.setStartPose(new Pose2d(-41, -157, 0));
        waitForStart();
        interfer();
        andrew.finish();
    }

    public static double x = 0;
    public static double y = 0;
    public static double r = 0;

    void interfer(){
        moveTo(new WayPoint(-20, -152, Math.toRadians(-25), 0.2));
        moveTo(new WayPoint(57, -20, Math.toRadians(-25), 1));

        while(opModeIsActive()){
            moveTo(new WayPoint(70, -20, Math.toRadians(0)));
        }
//        moveTo(new WayPoint(-41, -150, 0));
    }

    void moveTo(WayPoint target){
        andrew.reset_all();
        while(opModeIsActive() && !andrew.moveTo(target)){
            andrew.update();
        }
        andrew.stop_all();
    }
}
