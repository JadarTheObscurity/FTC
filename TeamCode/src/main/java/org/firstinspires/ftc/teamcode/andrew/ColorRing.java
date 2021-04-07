package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(group = "Andrew")
@Disabled
public class ColorRing extends OpMode {
    NormalizedColorSensor colorSensor;
    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    @Override
    public void loop() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("red", colors.red);
        telemetry.update();
    }
}
