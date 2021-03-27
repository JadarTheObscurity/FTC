package org.firstinspires.ftc.teamcode.johanson;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.WayPoint;

@Autonomous
@Disabled
public class JohansonAuto_2 extends LinearOpMode {
    Johanson johanson;
    @Override
    public void runOpMode() throws InterruptedException {
        johanson = new Johanson(hardwareMap);
        johanson.setStartPose(new Pose2d(-150, -150, 0));
        waitForStart();
        state_A();

        johanson.finish();
    }

    void state_A(){
        //Shoot
        moveTo(new WayPoint(-130, -10, Math.toRadians(-20)));
        shoot_three_ring();

        //put first wobble
        moveTo(new WayPoint(-120, 20, Math.toRadians(-150)));
        johanson.arm_down();

        //grab second wobble
        moveTo(new WayPoint(-75, -94, Math.toRadians(0)));

        //put second wobble
        moveTo(new WayPoint(-120, 10, Math.toRadians(-140)));
        johanson.arm_up();

        moveTo(new WayPoint(-130, -130, Math.toRadians(0)));
    }

    void moveTo(WayPoint target){
        johanson.reset_all();
        while(opModeIsActive() && !johanson.moveTo(target)){
            johanson.update();
        }
        johanson.stop_all();
    }

    void shoot_three_ring(){
        johanson.reset_all();
        while(opModeIsActive() && !johanson.shoot_3_ring()){

        }
    }


}
