package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class basicLineFollower extends LinearOpMode {

    private DriveTrain drive;
    private DriveVector vector;
    private SensorNetwork sensor;

    private double alpha;
    private double tile, tape, lineTarget, safety;

    @Override
    public void runOpMode(){
        sensor = new SensorNetwork(hardwareMap, this);
        drive = new DriveTrain(hardwareMap, this, sensor);
        vector = new DriveVector();


        tile = 0.32;
        tape = 0.84;
        lineTarget = 0.76;
        safety = 0.1;

        drive.initialize();
        vector.initialize();
        sensor.initialize();

        waitForStart();

        sensor.start();

        idle();
        vector.angle = 0;
        vector.mag = 0.1;


        while (opModeIsActive()){
//            alpha = sensor.getForwardColorsComponent(0);
//            if (alpha >= lineTarget - safety && alpha <= lineTarget + safety){
//                vector.mag = 0.1;
//                vector.angle = 0;
//            }else if (alpha < lineTarget && alpha > tile - safety) {
//                vector.mag = 0.1;
//                vector.angle = -Math.PI/4;
//            } else if (alpha > lineTarget && alpha < tape + safety){
//                vector.mag = 0.1;
//                vector.angle = Math.PI/4;
//            } else{
//                vector.mag = 0;
//            }
            idle();
            drive.goVector(vector, 0);
            telemetry.addData("alpha", alpha);
            telemetry.update();
        }
    }
}
