package org.firstinspires.ftc.teamcode.andrew;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.WayPoint;

@Autonomous(group = "Andrew")
//@Disabled
@Config
public class OdometryTest extends LinearOpMode {
    Andrew andrew;

    DcMotorEx ly, ry;
    @Override
    public void runOpMode() throws InterruptedException {
        andrew = new Andrew(hardwareMap);
        andrew.setStartPose(new Pose2d(0, 0, 0));
        ly = hardwareMap.get(DcMotorEx.class, "lf");
        ry = hardwareMap.get(DcMotorEx.class, "lb");
        ly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        andrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        test_odometry();
        andrew.finish();
    }

    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    void test_odometry(){
        moveTo(new WayPoint(x, y, Math.toRadians(r)));
    }

    void suck_test(){
        andrew.suck_spin();
        moveTo(new WayPoint(0, -100, 0, 4));
        moveTo(new WayPoint(0, -150, 0, 4));
    }

    void moveTo(WayPoint target){
        andrew.reset_all();
        while(opModeIsActive() && !andrew.moveTo(target)){
            andrew.update();
        }
        andrew.stop_all();
    }

}
