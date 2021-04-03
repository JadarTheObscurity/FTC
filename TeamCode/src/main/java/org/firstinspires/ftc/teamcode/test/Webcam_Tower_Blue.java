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
@TeleOp(name = "Webcam Blue Tower Tuner", group = "Test")
public class Webcam_Tower_Blue extends OpMode {
    FtcDashboard dashboard;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    Webcam webcam;

    public static int pipline_stage = 0;
    public static HSV_threshold threshold = TowerPipeline.blue_hsv.copy();
    public static HSV_threshold white_threshold = TowerPipeline.white_hsv.copy();

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        webcam = new Webcam(hardwareMap, "Webcam Tower", new Point(640, 480));
        webcam.setPipeline(pipeline);
        webcam.startStreaming(dashboard);
    }

    @Override
    public void loop() {
        TowerPipeline.curr_stage =pipline_stage;
        pipeline.setHSVThreshold(threshold);
        pipeline.setWhiteHSVThreshold(white_threshold);
    }
}
