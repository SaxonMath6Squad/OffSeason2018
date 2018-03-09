package org.firstinspires.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jothm on 3/8/2018.
 */

public class DriveEngine {
    DcMotor leftMotor;
    DcMotor rightMotor;
    HardwareMap hardwareMap;


    public DriveEngine(HardwareMap hw){
        hardwareMap = hw;
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
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void turn(double percentPower){
        leftMotor.setPower(percentPower);
        rightMotor.setPower(-percentPower);
    }
    public void brake(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
//    public void turn(double speed, double heading){
//        double distFromHeading = bloop-heading;
//
//    }
    public void kill(){
        brake();
    }
}
