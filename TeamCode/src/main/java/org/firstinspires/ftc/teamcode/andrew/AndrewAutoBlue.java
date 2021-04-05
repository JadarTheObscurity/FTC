package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.RingPipeline;
import org.firstinspires.ftc.teamcode.util.WayPoint;
import org.firstinspires.ftc.teamcode.util.Webcam;
import org.opencv.core.Point;

@Autonomous(group = "Andrew")
public class AndrewAutoBlue extends LinearOpMode {
    Andrew andrew;
    Webcam webcam;
    RingPipeline pipeline = new RingPipeline();
    RingPipeline.RingStatus ringStatus = RingPipeline.RingStatus.FOUR;
    @Override
    public void runOpMode() throws InterruptedException {

        andrew = new Andrew(hardwareMap);
        andrew.setStartPose(new Pose2d(-106, -157, 0));
        andrew.claw_grab();

        webcam = new Webcam(hardwareMap, "Webcam Ring", new Point(640, 480));
        webcam.setPipeline(pipeline);
        webcam.force_opne();
        webcam.startStreaming(Andrew.dashboard);
        telemetry.addData("Ready!!", "");
        telemetry.update();
        waitForStart();
        ringStatus = pipeline.ringStatus;
        telemetry.addData("Ring Status", ringStatus);
        telemetry.addData("White Pixel Num", pipeline.white_pixel);
        telemetry.update();
        switch (ringStatus){
            case NONE:
                state_A();
                break;
            case ONE:
                state_B();
                break;
            case FOUR:
                state_C();
                break;
        }
        andrew.finish();
    }



    void drop_intake(){

        moveTo(new WayPoint(-106, -145, 0, 0.2));
        moveTo(new WayPoint(-106, -160, 0, 0.2));
    }

    void go_home(){
//        moveTo(new WayPoint(-107, -140, Math.toRadians(0))); //Home
        moveTo(new WayPoint(-90, 20, Math.toRadians(0))); //White Line
    }

    void state_A(){

        //Put down yeh
        drop_intake();
        //Shoot
        moveTo(new WayPoint(-95, -20, Math.toRadians(-3)));
        shoot_ring(3, true);
        andrew.shooter_stop();

        //put first wobble
        andrew.arm_down();
        moveTo(new WayPoint(-120, 60, Math.toRadians(-90)));
        andrew.claw_release();

        //grab second wobble
        moveTo(new WayPoint(-70, -70, Math.toRadians(0)));
        moveTo(new WayPoint(-70, -85, Math.toRadians(0)));
        andrew.claw_grab();
        sleep(300);

        //put second wobble
        moveTo(new WayPoint(-115, 40, Math.toRadians(-90)));
        andrew.claw_release();
        sleep(200);
        andrew.arm_up();

        moveTo(new WayPoint(-110, 40, Math.toRadians(-90)));

        go_home();
    }

    void state_B(){
        //Put down yeh
        drop_intake();

        //Shoot
        andrew.shooter_shoot_tower();
        moveTo(new WayPoint(-92, -97, Math.toRadians(-3)));
        shoot_ring(3, true);


        //Shoot 2
        andrew.suck_spin();
        moveTo(new WayPoint(-95, -20, Math.toRadians(-4), 3));
        andrew.suck_spin();
        andrew.shooter_shoot_tower();
        sleep(800);
        shoot_ring(2, false);
        andrew.stop_all();

        //put first wobble
        andrew.arm_down();
        moveTo(new WayPoint(-85, 65, Math.toRadians(-180)));
        andrew.claw_release();

        //grab second wobble
        moveTo(new WayPoint(-70, -70, Math.toRadians(0)));
        moveTo(new WayPoint(-70, -87, Math.toRadians(0)));
        andrew.claw_grab();
        sleep(300);

        //put second wobble
        moveTo(new WayPoint(-80, 50, Math.toRadians(-180)));
        andrew.claw_release();
        andrew.arm_up();

        go_home();
    }

    void state_C(){

        //Put down yeh
        drop_intake();
        //Shoot
        moveTo(new WayPoint(-92, -97, Math.toRadians(-2)));
        shoot_ring(3, true);

        //Shoot 2
        andrew.suck_spin();
        moveTo(new WayPoint(-92, -72, Math.toRadians(-4), 2));
        andrew.suck_spin();
        andrew.shooter_shoot_tower();
        sleep(800);
        andrew.suck_stop();
        shoot_ring(2, false);

        andrew.suck_spin();
        moveTo(new WayPoint(-92, -20, Math.toRadians(-3), 2));
        andrew.suck_spin();
        andrew.shooter_shoot_tower();
        sleep(800);
        andrew.suck_stop();
        shoot_ring(3, false);
        andrew.stop_all();


        //put first wobble
        andrew.arm_down();
        moveTo(new WayPoint(-120, 160, Math.toRadians(-100)));
        andrew.claw_release();

        //grab second wobble
        moveTo(new WayPoint(-71, -70, Math.toRadians(0)));
        moveTo(new WayPoint(-71, -85, Math.toRadians(0)));
        andrew.claw_grab();
        sleep(500);

        //put second wobble
        moveTo(new WayPoint(-100, 160, Math.toRadians(-100)));
        andrew.claw_release();
        andrew.arm_up();

        go_home();
    }

    void moveTo(WayPoint target){
        andrew.reset_all();
        while(opModeIsActive() && !andrew.moveTo(target)){
            andrew.update();
        }
        andrew.stop_all();
    }


    void shoot_ring(int num, boolean pre_spin){
        andrew.reset_all();
        while(opModeIsActive() && !andrew.shoot_ring(num, pre_spin)){

        }
        andrew.load_down();
    }


}
