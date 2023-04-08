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

    private double heading;

    private AngularVelocity angularVelocity;
    
    /* CLASS CONSTRUCTOR */
    public SensorNetwork(HardwareMap hardwareMap, LinearOpMode opModeTool) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
        this.flag = true;
    }

    /**
     * Initalizes the IMU.
     */
    public void initialize(){
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 0, 90, 0, 0));

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.RADIANS);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    } //end init method

    /*
    *
    **/
    public void run(){
        while(opModeTool.opModeIsActive()){
            orientation = imu.getRobotYawPitchRollAngles();
            opModeTool.idle();
            opModeTool.idle();
            heading = orientation.getYaw(AngleUnit.RADIANS);
        }
    }

    public double getHeading(){
        return heading;
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