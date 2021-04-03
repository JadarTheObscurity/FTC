package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.WayPoint;

@Autonomous(group = "Andrew")
public class AndrewAutoRed extends LinearOpMode {
    Andrew andrew;
    @Override
    public void runOpMode() throws InterruptedException {
        andrew = new Andrew(hardwareMap);
        andrew.setStartPose(new Pose2d(150, -157, 0));
        andrew.claw_grab();
        waitForStart();
        test_move();
        andrew.finish();
    }

    void test_move(){
        andrew.setStartPose(new Pose2d(0, 0, 0));
        moveTo(new WayPoint(180, 180, Math.toRadians(0)));
        sleep(3000);
        moveTo(new WayPoint(0, 0, 0));
    }


    void state_A(){
        //Shoot
        moveTo(new WayPoint(130, -10, Math.toRadians(-10)));
        shoot_three_ring();

        //put first wobble
        andrew.arm_down();
        moveTo(new WayPoint(140, 0, Math.toRadians(150)));
        andrew.claw_release();

        //grab second wobble
        moveTo(new WayPoint(140, 0, Math.toRadians(-3)));
        moveTo(new WayPoint(50, -70, Math.toRadians(-3)));
        moveTo(new WayPoint(50, -82, Math.toRadians(-3)));
        andrew.claw_grab();
        sleep(300);

        //put second wobble
        moveTo(new WayPoint(140, -10, Math.toRadians(140)));
        andrew.claw_release();
        andrew.arm_up();

        moveTo(new WayPoint(130, -130, Math.toRadians(0)));
    }

    void moveTo(WayPoint target){
        andrew.reset_all();
        while(opModeIsActive() && !andrew.moveTo(target)){
            andrew.update();
        }
        andrew.stop_all();
    }

    void shoot_three_ring(){
        andrew.reset_all();
        while(opModeIsActive() && !andrew.shoot_3_ring()){

        }
    }


}
