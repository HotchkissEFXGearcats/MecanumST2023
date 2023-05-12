package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.PI;

// BEGIN CLASS //
public class DriveTrain {
    
    private HardwareMap hardwareMap;
    private LinearOpMode opModeTool;
    private SensorNetwork sensors;
    
    private ElapsedTime timer;
    
//    private Gyroscope imu;

    private DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    
    private int k;
    private int avgPosition, positionLF, positionRF, positionLB, positionRB;
    private int start, finish, getGoing, slowDown, driveBuffer;
    private double setHeading, driveHeading, botHeading, headingOffset, previous, duration;

    
    private double kp, kAuto;
    private double scaleTurn;
    private double turn;
    private boolean flag, warning;
    
    //private Headings headings; 
    
    /* CLASS CONSTRUCTOR */
    public DriveTrain(HardwareMap hardwareMap, LinearOpMode opModeTool, SensorNetwork sensors) {
        this.hardwareMap = hardwareMap;
        this.opModeTool = opModeTool;
        this.sensors = sensors;
        initialize();
    }
    
    public void initialize() {
        
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        
        //
        // Initialize Motors
        //
        
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        positionLF = leftFrontMotor.getCurrentPosition();
        positionRF = rightFrontMotor.getCurrentPosition();
        positionLB = leftBackMotor.getCurrentPosition();
        positionRB = rightBackMotor.getCurrentPosition();
        
        //
        // Initialize scalars and flags
        //
       
        flag = true;
        warning = false;
        
        driveHeading = 0.0;
        botHeading = 0.0;
        headingOffset = 0.0;
        previous = 0.0;
        duration = 0.0;
        
        kp = 0.35;
        kAuto = 0.4;
        scaleTurn = 1; //how fast the robot would go in driver mode
        flag = true;
        
        driveBuffer = 300;
        
    }  // end method initialize

    public int getPositionLF() {
        return leftFrontMotor.getCurrentPosition();
    }
    
    public int getPositionRF() {
        return rightFrontMotor.getCurrentPosition();
    }
    
    public int getPositionLB() {
        return leftBackMotor.getCurrentPosition();
    }
    
    public int getPositionRB() {
        return rightBackMotor.getCurrentPosition();
    }

    /**
     * Returns the motor position and has the capability to reset the tick counter if the param is true.
     * @param reset
     * @return motorPosition in ticks
     */
    public int linearPosition(boolean reset) {

        if (reset) {

            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
            rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } else {

            positionLF = leftFrontMotor.getCurrentPosition();
            positionRF = rightFrontMotor.getCurrentPosition();
            positionLB = leftBackMotor.getCurrentPosition();
            positionRB = rightBackMotor.getCurrentPosition();

            avgPosition = (int)((abs(positionLF) + abs(positionRF) + abs(positionLB) + abs(positionRB))/4);

        } // end if-else

        return avgPosition;

    }  // end method drivePosition

    /*
          Robot Movement Functions
          1.testMotors - tests motors and vectors (in radians)
          2.autonVector - Moves the robot in a vector for a certain amt of ticks.Takes the vector in degrees.
          3.autonVectorTurn - Moves the robot in a vector and turns for certain amt of ticks.Uses degrees.
          4.TurnTo - Turns the robot in degrees
     */

    public void testMotors(DriveVector vector) {
        leftFrontMotor.setPower(vector.mag * sin((vector.angle + PI/4)) );  //yPower
        rightFrontMotor.setPower(vector.mag * sin((vector.angle - PI/4)) );  //yPower
        leftBackMotor.setPower(vector.mag * sin((vector.angle - PI/4)) );  //xPower
        rightBackMotor.setPower(vector.mag * sin((vector.angle + PI/4)) );  //xPower
    }

    /**
     * Moves robot with a certain vector (degrees) for a set amount of ticks.
     * @param vector
     * @param toPosition
     * @return warning
     */
    public boolean autonVector(DriveVector vector, int toPosition) {
        if (vector.mag < 0.05) {
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
        } else {
            while (abs(linearPosition(false)) < toPosition) {
                botHeading = sensors.getHeading();
                headingOffset = (botHeading - setHeading);
                if (abs(headingOffset) < PI/4) {
                    turn = (kAuto * headingOffset) / (PI/4);
                } else if (abs(headingOffset) < PI/3) {
                    turn = kAuto * 1;
                } else {
                    leftFrontMotor.setPower(0.0);
                    rightFrontMotor.setPower(0.0);
                    leftBackMotor.setPower(0.0);
                    rightBackMotor.setPower(0.0);
                    warning = true;
                }  // end if-else turn < 0.05 case is not turning
                leftFrontMotor.setPower(vector.mag * sin(((vector.angle*PI/180) - setHeading)+PI/4) - turn );  //yPower
                leftBackMotor.setPower(vector.mag * -sin(((vector.angle*PI/180) - setHeading)-PI/4) - turn );  //xPower
                rightFrontMotor.setPower(vector.mag * -sin(((vector.angle*PI/180) - setHeading)-PI/4) + turn);  //yPower
                rightBackMotor.setPower(vector.mag * sin(((vector.angle*PI/180) - setHeading)+PI/4) + turn);  //xPower
                /*opModeTool.telemetry.addData("Position: ", linearPosition(false));
                opModeTool.telemetry.update();*/
            }  // end while
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);
            linearPosition(true);
        }  // end if-else
        return warning;
    }  // end method autonVector

    /**
     * Auton Vector Turn. AutonVector, but with turning.
     * @param vector
     * @param toPosition
     * @param turn
     * @return
     */
    public boolean autonVectorTurn(DriveVector vector, int toPosition, double turnPower, double toHeading) {
        botHeading = sensors.getHeading();
        toHeading = PI/180 * toHeading;
        double turnRight = 0;
        double turnLeft = 0;
        opModeTool.telemetry.addData("check", toHeading);
        opModeTool.telemetry.update();

        while (abs(linearPosition(false)) < toPosition) {
            if (abs(toHeading-botHeading) > PI/180 ) {
                if (toHeading-botHeading > 0) {
                    turnLeft = turnPower;
                    turnRight = -turnPower;
                }
                else{
                    turnLeft = -turnPower;
                    turnRight = turnPower;
                }
            }
            else{
                turnLeft = 0;
                turnRight = 0;
            }
            botHeading = sensors.getHeading();
            leftFrontMotor.setPower(vector.mag * sin(((vector.angle*PI/180) - botHeading)+PI/4) + turnLeft);  //yPower
            leftBackMotor.setPower(vector.mag * -sin(((vector.angle*PI/180) - botHeading)-PI/4) + turnLeft);  //xPower
            rightFrontMotor.setPower(vector.mag * -sin(((vector.angle*PI/180) - botHeading)-PI/4) + turnRight);  //yPower
            rightBackMotor.setPower(vector.mag * sin(((vector.angle*PI/180) - botHeading)+PI/4) + turnRight);  //xPower
            opModeTool.telemetry.addData("turnLeft: ", turnLeft);
            opModeTool.telemetry.addData("turnRight: ", turnRight);
            opModeTool.telemetry.update();
        }  // end while
        while (abs(toHeading-botHeading) > PI/180 ) {
            if (toHeading-botHeading > 0) {
                leftFrontMotor.setPower(turnPower);
                leftBackMotor.setPower(turnPower);
                rightFrontMotor.setPower(-turnPower);
                rightBackMotor.setPower(-turnPower);
            }
            else{
                leftFrontMotor.setPower(-turnPower);
                leftBackMotor.setPower(-turnPower);
                rightFrontMotor.setPower(turnPower);
                rightBackMotor.setPower(turnPower);
            }
        }
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        linearPosition(true);
        return warning;
    }  // end method autonVectorTurn


    /**
     * Stops bot
     * @return avgPosition
     */
    public int stop() {
        
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        
        positionLF = leftFrontMotor.getCurrentPosition();
        positionRF = rightFrontMotor.getCurrentPosition();
        positionLB = leftBackMotor.getCurrentPosition();
        positionRB = rightBackMotor.getCurrentPosition();
            
        avgPosition = (int)((abs(positionLF) + abs(positionRF) + abs(positionLB) + abs(positionRB))/4);
        return avgPosition;
    }

        /*
    public double AutonVectorAccel(DriveVector vector, int toPosition){

    }*/


    /**
     * turnTo (in degrees)
     * @param turnPower
     * @param toHeading
     * @return
     */
    public double turnTo(double turnPower, double toHeading){
        botHeading = sensors.getHeading();
        toHeading = PI/180 * toHeading;
        while (abs(toHeading-botHeading) > PI/180 ) {
            if (toHeading-botHeading > 0) {
                leftFrontMotor.setPower(turnPower);
                leftBackMotor.setPower(turnPower);
                rightFrontMotor.setPower(-turnPower);
                rightBackMotor.setPower(-turnPower);
            }
            else{
                leftFrontMotor.setPower(-turnPower);
                leftBackMotor.setPower(-turnPower);
                rightFrontMotor.setPower(turnPower);
                rightBackMotor.setPower(turnPower);
            }
            botHeading = sensors.getHeading();
            opModeTool.telemetry.addData("botHeading: ", botHeading);
            opModeTool.telemetry.update();

        }  // end while
        /*psudo code for correction
        make a special case if target is 180
        for the rest, make botheading - setheading and correct.
         */
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        setHeading = toHeading;
        return botHeading;
    }  // end method turn
    public void resetHeading(){
        setHeading = sensors.getHeading();
    }
    public void goVector(DriveVector vector, double turnPower){
        turnPower = -turnPower;
        if (vector.mag < 0.05) {
            if (abs(turnPower) < 0.05) {
                leftFrontMotor.setPower(0.0);
                rightFrontMotor.setPower(0.0);
                leftBackMotor.setPower(0.0);
                rightBackMotor.setPower(0.0);
                setHeading = sensors.getHeading();
            } else {

                leftFrontMotor.setPower(turnPower* scaleTurn);
                leftBackMotor.setPower(turnPower* scaleTurn);
                rightFrontMotor.setPower(-turnPower* scaleTurn);
                rightBackMotor.setPower(-turnPower* scaleTurn);
                setHeading = sensors.getHeading();
            }

        } else {

            if (abs(turnPower) < 0.05) {
                botHeading = sensors.getHeading();
                headingOffset = (botHeading - setHeading);
                if (abs(headingOffset) < PI/4) {
                    turn = (kp * headingOffset) / (PI/4);
                } else if (abs(headingOffset) < PI/3) {
                    turn = kp * 1;
                } else {
                    turn = 0;
                }  // end if-else

                leftFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading)+PI/4) + turn );  //yPower
                leftBackMotor.setPower(vector.mag * -sin((vector.angle - setHeading)-PI/4) + turn );  //xPower
                rightFrontMotor.setPower(vector.mag * -sin((vector.angle - setHeading)-PI/4) - turn);  //yPower
                rightBackMotor.setPower(vector.mag * sin((vector.angle - setHeading)+PI/4) - turn);  //xPower

            } else {
                // drive and turn at the same time
                turn = turnPower * scaleTurn;

                leftFrontMotor.setPower(vector.mag * sin((vector.angle - setHeading)+PI/4) + turn );  //yPower
                leftBackMotor.setPower(vector.mag * -sin((vector.angle - setHeading)-PI/4) + turn );  //xPower
                rightFrontMotor.setPower(vector.mag * -sin((vector.angle - setHeading)-PI/4) - turn);  //yPower
                rightBackMotor.setPower(vector.mag * sin((vector.angle - setHeading)+PI/4) - turn);  //xPower

                setHeading = sensors.getHeading();

            }

        }
    }

}  // end class







