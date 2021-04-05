package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.RingPipeline;
import org.firstinspires.ftc.teamcode.util.WayPoint;
import org.firstinspires.ftc.teamcode.util.Webcam;
import org.opencv.core.Point;

@Autonomous(group = "Andrew")
public class AndrewAutoRed extends LinearOpMode {
    Andrew andrew;
    Webcam webcam;
    RingPipeline pipeline = new RingPipeline();
    RingPipeline.RingStatus ringStatus = RingPipeline.RingStatus.FOUR;
    @Override
    public void runOpMode() throws InterruptedException {

        andrew = new Andrew(hardwareMap);
        andrew.setStartPose(new Pose2d(106, -157, 0));
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

        moveTo(new WayPoint(106, -145, 0, 0.2));
        moveTo(new WayPoint(106, -160, 0, 0.2));
    }

    void go_home(){
//        moveTo(new WayPoint(107, -140, Math.toRadians(0))); // Home
        moveTo(new WayPoint(90, 20, Math.toRadians(0))); //White Line
    }

    void state_A(){

        //Put down yeh
        drop_intake();
        //Shoot
        moveTo(new WayPoint(90, -20, Math.toRadians(-4)));
        shoot_ring(3, true);
        andrew.shooter_stop();

        //put first wobble
        andrew.arm_down();
        moveTo(new WayPoint(120, 30, Math.toRadians(90)));
        andrew.claw_release();

        //grab second wobble
        moveTo(new WayPoint(42, -70, Math.toRadians(0)));
        moveTo(new WayPoint(42, -85, Math.toRadians(0)));
        andrew.claw_grab();
        sleep(300);

        //put second wobble
        moveTo(new WayPoint(100, 20, Math.toRadians(90)));
        andrew.claw_release();
        andrew.arm_up();

        go_home();
    }

    void state_B(){
        //Put down yeh
        drop_intake();

        //Shoot
//        moveTo(new WayPoint(107, -100, Math.toRadians(-6)));
        moveTo(new WayPoint(85, -97, Math.toRadians(-6), 2));
        shoot_ring(2, true);


        //Shoot 2
        andrew.suck_spin();
        moveTo(new WayPoint(85, -20, Math.toRadians(-5), 3));
        andrew.suck_spin();
        andrew.shooter_shoot_tower();
        sleep(800);
        shoot_ring(3, false);
        andrew.stop_all();

        //put first wobble
        andrew.arm_down();
        moveTo(new WayPoint(115, 65, Math.toRadians(-180)));
        andrew.claw_release();

        //grab second wobble
        moveTo(new WayPoint(52, -70, Math.toRadians(0)));
        moveTo(new WayPoint(52, -87, Math.toRadians(0)));
        andrew.claw_grab();
        sleep(300);

        //put second wobble
        moveTo(new WayPoint(120, 50, Math.toRadians(-180)));
        andrew.claw_release();
        andrew.arm_up();

        go_home();
    }

    void state_C(){

        //Put down yeh
        drop_intake();
        //Shoot
//        moveTo(new WayPoint(107, -110, Math.toRadians(-6)));
        moveTo(new WayPoint(92, -100, Math.toRadians(-3.5), 1));
        shoot_ring(3, true);


        //Shoot 2
        andrew.suck_spin();
        moveTo(new WayPoint(92, -75, Math.toRadians(-3), 1.7));
        andrew.suck_spin();
        andrew.shooter_shoot_tower();
        sleep(800);
        shoot_ring(2, false);
        andrew.stop_all();

        andrew.suck_spin();
        moveTo(new WayPoint(91, -20, Math.toRadians(-3), 3));
        andrew.suck_spin();
        andrew.shooter_shoot_tower();
        sleep(800);
        andrew.suck_reverse(0.2);
        shoot_ring(2, false);
        andrew.shooter_stop();
        andrew.stop_all();


        //put first wobble
        andrew.arm_down();
        moveTo(new WayPoint(100, 160, Math.toRadians(90)));
        andrew.claw_release();

        //grab second wobble;
        moveTo(new WayPoint(50, -65, Math.toRadians(0)));
        moveTo(new WayPoint(50, -87, Math.toRadians(0)));
        andrew.claw_grab();
        sleep(500);

        //put second wobble
        moveTo(new WayPoint(90, 150, Math.toRadians(90)));
        moveTo(new WayPoint(90, 150, Math.toRadians(90)));
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
