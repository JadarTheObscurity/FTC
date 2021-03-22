package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.HSV_threshold;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;
import org.opencv.core.Point;

@Config
@TeleOp(name = "Webcam Tower Tuner", group = "Test")
public class Webcam_Tower extends OpMode {
    FtcDashboard dashboard;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    Webcam webcam;

    public static int pipline_stage = 0;
    public static int target = 0;
    public static HSV_threshold threshold = new HSV_threshold(0, 180, 0, 255, 0, 255);

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        webcam = new Webcam(hardwareMap, new Point(640, 480));
        webcam.setPipeline(pipeline);
        webcam.startStreaming(dashboard);
    }

    @Override
    public void loop() {
        TowerPipeline.curr_stage =pipline_stage;
        pipeline.setHSVThreshold(threshold);
    }
}
