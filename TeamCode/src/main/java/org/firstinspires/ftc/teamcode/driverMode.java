package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class driverMode extends LinearOpMode{

    private SensorNetwork sensor;
    private DriveTrain drive;
    private DriveVector vector;
    private double heading, pitch, roll;

    @Override
    public void runOpMode(){
        sensor = new SensorNetwork(hardwareMap, this);
        drive = new DriveTrain(hardwareMap, this, sensor);
        vector = new DriveVector();

        sensor.initialize();
        drive.initialize();
        vector.initialize();

        waitForStart();

        sensor.start();

        while(opModeIsActive()){
            vector = vector.makeVector(gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Mag: ", vector.mag);
            telemetry.addData("Angle: ", vector.angle);
            telemetry.update();
            drive.goVector(vector, gamepad1.left_stick_x);
            if(gamepad1.a){
                drive.resetHeading();
                telemetry.addData("I did it: ", drive.setHeading);
                telemetry.update();
            }
        }// end while loop
    }
}
