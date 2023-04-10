package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class TestingIMU extends LinearOpMode{

    private SensorNetwork sensor;
    private double heading;

    @Override
    public void runOpMode(){
        sensor = new SensorNetwork(hardwareMap, this);

        sensor.initialize();

        heading = sensor.getHeading();

        telemetry.addData("Heading: ", "%.05f", heading);
        telemetry.addLine();
        telemetry.update();

        waitForStart();

        sensor.start();

        while(opModeIsActive()){

            heading = sensor.getHeadingDeg();

            telemetry.addData("Heading: ", "%.05f", heading);
            telemetry.addLine();
            telemetry.update();
            idle();
        }// end while loop
    }
}
