package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Webcam {
    OpenCvCamera webcam;
    boolean isStreaming = false;
    public Point resolution;
    public Webcam(HardwareMap hardwareMap, String name, Point resolution){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name), cameraMonitorViewId);
        this.resolution = resolution;
    }

    public OpenCvCamera getWebcam(){
        return webcam;
    }

    public void setPipeline(OpenCvPipeline pipeline){
        webcam.setPipeline(pipeline);
    }

    public void startStreaming(FtcDashboard dashboard){
        if(!isStreaming) {
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming((int) resolution.x, (int) resolution.y, OpenCvCameraRotation.UPRIGHT);
                }
            });
            dashboard.startCameraStream(webcam, 30);
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
