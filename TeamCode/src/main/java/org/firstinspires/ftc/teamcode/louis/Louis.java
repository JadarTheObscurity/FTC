package org.firstinspires.ftc.teamcode.louis;

import android.media.MediaExtractor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.JadarControl;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;
import org.opencv.core.Point;

public class Louis {
    public MecanumDriveTrain driveTrain;
    JadarControl control;
    Webcam webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    FtcDashboard dashboard;

    public Louis(HardwareMap hardwareMap){
        driveTrain = new MecanumDriveTrain(hardwareMap, true);
        control = new JadarControl(driveTrain);
        webcam = new Webcam(hardwareMap, new Point(640, 480));
    }


    //Drive Train Behavior
    public boolean moveTo(double x, double y, double r){
        return control.moveTo(x, y, r);
    }

    public void reset_pos(){
        driveTrain.reset_pos();
        control.reset();
    }

    public void move(double x, double y, double r){
        driveTrain.move(x, y, r);
    }
}
