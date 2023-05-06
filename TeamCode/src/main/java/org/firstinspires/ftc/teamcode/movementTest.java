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
        vector.angle = 45;
        vector.mag = 0.3;
        drive.autonVectorTurn(vector,1000,0.4,-90);


        while(opModeIsActive()){

            heading = sensor.getHeadingDeg();
            roll = sensor.getRollDeg();
            pitch = sensor.getPitchDeg();
            idle();


        }// end while loop
    }
}
