package org.firstinspires.ftc;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jothm on 3/8/2018.
 */

public class DriveEngine {
    DcMotor leftMotor;
    DcMotor rightMotor;
    GyroSensor orientation;
    PIDController turnController;
    HardwareMap hardwareMap;
    private final int TICKS_PER_REV = 560;
    private final double DRIVE_WHEEL_DIAMETER_INCHES = 4.0;
    private final double HEADING_THRESHOLD = 2.0;
    private final double TURN_KP = 0.05; // gain * error
    private final double TURN_KI = 0.01; // gain * error * time(some decimal of seconds)
    private final double TURN_KD = 0;

    public DriveEngine(HardwareMap hw){
        hardwareMap = hw;
        turnController = new PIDController(TURN_KP, TURN_KI, TURN_KD);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double power){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void turn(double percentPower){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(percentPower);
        rightMotor.setPower(-percentPower);
    }
    public void brake(){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void driveDistance(int tick){
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(tick);
        rightMotor.setTargetPosition(tick);
        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }
    public void driveDistance(double distanceInInches, double power){
        int targetDistanceTicks = (int)(TICKS_PER_REV*distanceInInches/ DRIVE_WHEEL_DIAMETER_INCHES +0.5);
        Log.d("Target Distance Inches", Double.toString(distanceInInches));
        Log.d("Target Distance Ticks", Integer.toString(targetDistanceTicks));
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(targetDistanceTicks);
        rightMotor.setTargetPosition(targetDistanceTicks);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void turn(double heading, LinearOpMode mode){
        turnController.setSp(0);
        double power;
        double distFromHeading = heading - orientation.getHeading();
        // makes negative counter-clockwise and positive clockwise
        if(distFromHeading > 180) distFromHeading -= 360;
        else if(distFromHeading < -180) distFromHeading += 360;

        while (mode.opModeIsActive() && Math.abs(distFromHeading) > HEADING_THRESHOLD){
            power = turnController.calculatePID(distFromHeading);
            turn(power);
            mode.sleep(5);
            distFromHeading = heading - orientation.getHeading();
            if(distFromHeading > 180) distFromHeading -= 360;
            else if(distFromHeading < -180) distFromHeading += 360;
        }
        brake();
    }
    public void kill(){
        brake();
    }
}
