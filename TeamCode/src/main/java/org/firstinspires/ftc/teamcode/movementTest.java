package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class movementTest extends LinearOpMode{

    private SensorNetwork sensor;
    private DriveTrain drive;
    private DriveVector vector;

    private double heading, roll, pitch;

    @Override
    public void runOpMode(){
        sensor = new SensorNetwork(hardwareMap, this);
        drive = new DriveTrain(hardwareMap, this, sensor);
        vector = new DriveVector();

        sensor.initialize();
        vector.initialize();
        drive.initialize();

        waitForStart();

        sensor.start();

        idle();
        drive.linearPosition(true);
        /*vector.angle = 0.6;
        vector.mag = 0.4;
        drive.autonVector(vector, 1000);*/
        drive.LturnTo(0.4, 90);


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
