package org.firstinspires.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by amdag on 3/15/2018.
 */

public class HolonomicDriveEngine {
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    HardwareMap hardwareMap;
public HolonomicDriveEngine(HardwareMap hw) {
    hardwareMap = hw;
    frontLeft = hardwareMap.dcMotor.get("frontLeft");
    backLeft = hardwareMap.dcMotor.get("backLeft");
    frontRight = hardwareMap.dcMotor.get("frontRight");
    backRight = hardwareMap.dcMotor.get("backRight");
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}
public void driveOnHeading(double heading){
    frontLeft.setPower(Math.sin(Math.toRadians(heading+45)));
    backLeft.setPower(Math.cos(Math.toRadians(heading+45)));
    frontRight.setPower(Math.sin(Math.toRadians(heading+45)));
    backRight.setPower(Math.cos(Math.toRadians(heading+45)));
}
public void turn(double power){
    frontLeft.setPower(power);
    backLeft.setPower(power);
    frontRight.setPower(-power);
    backRight.setPower(-power);
}

}


