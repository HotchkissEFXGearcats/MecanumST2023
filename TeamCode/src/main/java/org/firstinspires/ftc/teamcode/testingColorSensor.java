package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class testingColorSensor extends LinearOpMode{

    private SensorNetwork sensor;

    private NormalizedRGBA allColor;
    private float alpha;

    @Override
    public void runOpMode(){
        sensor = new SensorNetwork(hardwareMap, this);
        sensor.initialize();
        waitForStart();

        while (opModeIsActive()){
            allColor = sensor.getForwardColors();
            alpha = sensor.getForwardColorsComponent(0);
            telemetry.addData("color", allColor.alpha);
            telemetry.addLine();
            telemetry.addData("componentAlpha", alpha);
            telemetry.update();
        }
    }

}
