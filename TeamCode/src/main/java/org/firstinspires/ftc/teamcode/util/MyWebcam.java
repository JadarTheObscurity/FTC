package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class MyWebcam {
    OpenCvCamera webcam;
    boolean isStreaming = false;
    public Point resolution;
    public MyWebcam(HardwareMap hardwareMap, Point resolution){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.resolution = resolution;
    }

    public void setPipeline(OpenCvPipeline pipeline){
        webcam.setPipeline(pipeline);
    }

    public void startStreaming(FtcDashboard dashboard){
        if(!isStreaming) {
            dashboard.startCameraStream(webcam, 30);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming((int) resolution.x, (int) resolution.y, OpenCvCameraRotation.UPRIGHT);
                }
            });

            isStreaming = true;
        }
    }

    public void stopStreaming(){
        if(isStreaming) {
            webcam.stopStreaming();
            isStreaming = false;
        }
    }


}
