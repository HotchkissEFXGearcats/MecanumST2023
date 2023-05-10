package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class SensorNetwork extends Thread{
    
    private HardwareMap hardwareMap;
    private NormalizedColorSensor forwardColor;
    private NormalizedRGBA forwardColors;
    private LinearOpMode opModeTool;
    private ElapsedTime timer;

    public boolean flag;

    private IMU imu;
    private YawPitchRollAngles orientation;

    private double heading, pitch, roll, zeroHeading, zeroRoll, zeroPitch;


    private AngularVelocity angularVelocity;
    
    /* CLASS CONSTRUCTOR */

    /**
     * Default constructor
     * @param hardwareMap
     * @param opModeTool
     */
    public SensorNetwork(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
        this.flag = true;
        this.zeroHeading = 0.0;
    }

    /**
     * Initalizes the IMU.
     */
    public void initialize(){
        this.zeroPitch = 0.0;
        this.zeroRoll = 0.0;
       // zeroHeading = 0.0;
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 0, 0, 0, 0));

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        opModeTool.idle();
        orientation = imu.getRobotYawPitchRollAngles();
        opModeTool.idle();
        opModeTool.idle();
        zeroHeading = orientation.getYaw(AngleUnit.RADIANS);

        forwardColor = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        forwardColor.setGain(0.5F);


/*
        for( int i = 0; i<5; i++){
            this.orientation = this.imu.getRobotYawPitchRollAngles();
            opModeTool.telemetry.addData("Loop i: ", i);
            for (int j = 0; j < 10; j++){
                opModeTool.idle();
                opModeTool.telemetry.addData("Idle j: ", j);
            }
            this.zeroHeading += this.orientation.getYaw(AngleUnit.RADIANS);
            this.zeroRoll += this.orientation.getRoll(AngleUnit.RADIANS);
            this.zeroPitch += this.orientation.getPitch(AngleUnit.RADIANS);
            opModeTool.telemetry.addData("Yaw reading: ", "%.05f", this.orientation.getYaw(AngleUnit.RADIANS));
        }
        this.zeroHeading = this.zeroHeading/5.0;
        this.zeroRoll = this.zeroRoll/5.0;
        this.zeroPitch = this.zeroPitch/5.0;
        opModeTool.telemetry.addData("zeroPitch: ", "%.05f", this.zeroPitch);
        opModeTool.telemetry.addData("zeroRoll: ", "%.05f", this.zeroRoll);
        opModeTool.telemetry.addData("zeroHeading: ", "%.05f", this.zeroHeading);
        opModeTool.telemetry.update();

 */
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    } //end init method

    /**
    *runs sensornetwork readings in a separate thread
    **/
    public void run(){
        /*zeroPitch = 0.0;
        zeroRoll = 0.0;
        zeroHeading = 0.0;
        for( int i = 0; i<5; i++){
            orientation = imu.getRobotYawPitchRollAngles();
            for (int j = 0; j < 10; j++){
                opModeTool.idle();
            }
            opModeTool.telemetry.addData("ZeroHeading: ", "%.05f", zeroHeading);
            opModeTool.telemetry.update();
            zeroHeading += -orientation.getYaw(AngleUnit.RADIANS);
            zeroRoll += orientation.getRoll(AngleUnit.RADIANS);
            zeroPitch += orientation.getPitch(AngleUnit.RADIANS);
        }
        zeroHeading = zeroHeading/5.0;
        zeroRoll = zeroRoll/5.0;
        zeroPitch = zeroPitch/5.0;*/
        imu.resetYaw();
        while(opModeTool.opModeIsActive()){
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            opModeTool.idle();
            heading = -orientation.getYaw(AngleUnit.RADIANS);
            pitch = orientation.getPitch(AngleUnit.RADIANS);
            roll = orientation.getRoll(AngleUnit.RADIANS);
        }
    }  // end run method

    public double getHeading(){return heading;}
    public double getHeadingDeg(){return heading*180/Math.PI;}

    public double getPitch(){return pitch;}
    public double getPitchDeg(){
        return pitch*180/Math.PI;
    }

    public double getRoll(){return roll;}
    public double getRollDeg(){
        return roll*180/Math.PI;
    }

    /**
     * @return heading in degrees
     */
    /*
    public double getHeading(){
        return heading - zeroHeading;
    }
    public double getRawHeading(){return heading;}
    public double getHeadingDeg(){return (heading - zeroHeading)*180/Math.PI;}

    public double getPitch(){return pitch - zeroPitch;}
    public double getPitchDeg(){
        return (pitch - zeroPitch)*180/Math.PI;
    }

    public double getRoll(){return roll - zeroRoll;}
    public double getRollDeg(){
        return (roll - zeroRoll)*180/Math.PI;
    }*/

    public NormalizedRGBA getForwardColors() {
        forwardColors = forwardColor.getNormalizedColors();
        opModeTool.idle();
        return forwardColors;
    }


    /*
    public void initialize(float gainForward, float gainFront, float gainRear) {
        forwardColor = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        forwardColor.setGain(gainForward);
    }

    public NormalizedRGBA getForwardColors() {
        forwardColors = forwardColor.getNormalizedColors();
        opModeTool.idle();
        return forwardColors;
    }
    
    public void getColors() {
        forwardColors = forwardColor.getNormalizedColors();
    }
    */
    
}  // end class