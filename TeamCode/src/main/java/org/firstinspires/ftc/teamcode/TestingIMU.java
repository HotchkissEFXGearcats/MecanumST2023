package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class TestingIMU extends LinearOpMode{

    private SensorNetwork sensor;
    private double heading, pitch, roll;

    @Override
    public void runOpMode(){
        sensor = new SensorNetwork(hardwareMap, this);

        sensor.initialize();



        waitForStart();

        sensor.start();

        while(opModeIsActive()){

            heading = sensor.getHeadingDeg();
            roll = sensor.getRollDeg();
            pitch = sensor.getPitchDeg();

            telemetry.addData("Heading: ", "%.05f", heading);
            telemetry.addLine();
            telemetry.addData("Roll: ", "%.05f", roll);
            telemetry.addLine();
            telemetry.addData("Pitch: ", "%.05f", pitch);
            telemetry.addLine();
            telemetry.update();
            idle();
        }// end while loop
    }
}
