package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

class TestingIMU extends LinearOpMode{

    private SensorNetwork sensor;
    private double heading;

    @Override
    public void runOpMode(){
        sensor = new SensorNetwork(hardwareMap, this);
        waitForStart();

        sensor.start();

        while(opModeIsActive()){

            heading = sensor.getHeading();

            telemetry.addData("Heading: ", "%.01f", heading);
            telemetry.addLine();
            telemetry.update();
            idle();
        }// end while loop
    }
}
