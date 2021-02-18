package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.MyWebcam;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@TeleOp(name = "Webcam Tower", group = "Test")
public class Webcam_Tower extends OpMode {
    FtcDashboard dashboard;
    TowerPipeline pipeline = new TowerPipeline();
    MyWebcam webcam;

    public static int pipline_stage = 0;
    public static int min_h = 10;
    public static int max_h = 30;
    public static int min_s = 30;
    public static int max_s = 255;
    public static int min_v = 230;
    public static int max_v = 255;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        webcam = new MyWebcam(hardwareMap, new Point(640, 480));
        webcam.setPipeline(pipeline);
        webcam.startStreaming(dashboard);
    }

    @Override
    public void loop() {
        TowerPipeline.curr_stage =pipline_stage;
        pipeline.setHSVThreshold(min_h, max_h, min_s, max_s, min_v, max_v);
    }
}
