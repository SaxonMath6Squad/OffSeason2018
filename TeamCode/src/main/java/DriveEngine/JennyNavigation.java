package DriveEngine;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.InputStream;

import Autonomous.HeadingVector;
import Autonomous.Location;
import MotorControllers.JsonConfigReader;
import MotorControllers.NewMotorController;
import MotorControllers.PIDController;
import SensorHandlers.ImuHandler;


/**
 * Created by Jeremy on 8/23/2017.
 */

public class JennyNavigation extends Thread{
    public static final int NORTH = 0;
    public static final int SOUTH = 180;
    public static final int WEST = 270;
    public static final int EAST = 90;
    public NewMotorController[] driveMotors = new NewMotorController[4];
    public static final int FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR = 0;
    public static final int FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_HOLONOMIC_DRIVE_MOTOR = 3;
    private volatile long threadDelayMillis = 50;
    public volatile double robotHeading = 0;
    private volatile double [] lastMotorPositionsInInches = {0,0,0,0};
    private PIDController headingController, turnController;
    private Location myLocation;

    public ImuHandler orientation;
    private long timeAtAccelerationStart;
    public volatile boolean shouldRun = true;

    private final double HEADING_THRESHOLD = 3;
    private final double WHEEL_BASE_RADIUS = 20;
    private final double FL_WHEEL_HEADING_OFFSET = 45;
    private final double FR_WHEEL_HEADING_OFFSET = 315;
    private final double BR_WHEEL_HEADING_OFFSET = 45;
    private final double BL_WHEEL_HEADING_OFFSET = 315;
    private double acceleration = 0;
    private HardwareMap hardwareMap;
    public JennyNavigation(HardwareMap hw, String configFile) throws Exception{
        //initialize driveMotors
        //initialize sensors
        hardwareMap = hw;
        initializeUsingConfigFile(configFile);
        orientation = new ImuHandler("imu", hardwareMap);
//        myLocation.setX(0);
//        myLocation.setY(0);
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (shouldRun) {
                    updateData();
                    safetySleep(threadDelayMillis);
                }
            }
        });

    }

    private void updateLastMotorPositionsInInches(){
        for (int i = 0; i < driveMotors.length; i++){
            lastMotorPositionsInInches[i] = driveMotors[i].getInchesFromStart();
        }
    }

    private void updateHeading(){
        robotHeading = orientation.getOrientation();
    }

    private void updateData(){
        updateHeading();
        HeadingVector [] wheelVectors = getWheelVectors();
        HeadingVector travelVector = wheelVectors[0].addVectors(wheelVectors);
        double headingOfRobot = travelVector.getHeading();
        double magnitudeOfRobot = travelVector.getMagnitude();
        double actualHeading = headingOfRobot + robotHeading;
        HeadingVector actualVector = new HeadingVector();
        actualVector.calculateVector(actualHeading, magnitudeOfRobot);
        double deltaX = actualVector.x();
        double deltaY = actualVector.y();
        myLocation.addXY(deltaX, deltaY);
        updateLastMotorPositionsInInches();
    }

    private void safetySleep(long time){
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < time && shouldRun);
    }

    public void setThreadDelayMillis(long delayMillis){
        threadDelayMillis = delayMillis;
    }

    public double getOrientation(){
        return robotHeading;
    }

    public void initializeUsingConfigFile(String file){
        InputStream stream = null;
        try {
            stream = hardwareMap.appContext.getAssets().open(file);
        }
        catch(Exception e){
            Log.d("Drive Engine Error: ",e.toString());
            throw new RuntimeException("Drive Engine Open Config File Fail: " + e.toString());
        }
        JsonConfigReader reader = new JsonConfigReader(stream);
        try{
            driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = new NewMotorController(reader.getString("FRONT_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new NewMotorController(reader.getString("FRONT_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = new NewMotorController(reader.getString("BACK_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new NewMotorController(reader.getString("BACK_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            for (int i = 0; i < driveMotors.length; i++) {
                driveMotors[i].setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("BRAKE")){
                for (int i = 0; i < driveMotors.length; i++) {
                    driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            else if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("FLOAT")){
                for (int i = 0; i < driveMotors.length; i++) {
                    driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
            if(reader.getString("FRONT_LEFT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("FRONT_LEFT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("FRONT_RIGHT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("FRONT_RIGHT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("BACK_RIGHT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("BACK_RIGHT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("BACK_LEFT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("BACK_LEFT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorDirection(DcMotorSimple.Direction.FORWARD);
            }
            headingController = new PIDController(reader.getDouble("HEADING_Kp"), reader.getDouble("HEADING_Ki"), reader.getDouble("HEADING_Kd"));
            headingController.setIMax(reader.getDouble("HEADING_Ki_MAX"));
            turnController = new PIDController(reader.getDouble("TURN_Kp"), reader.getDouble("TURN_Ki"), reader.getDouble("TURN_Kd"));
            turnController.setIMax(reader.getDouble("TURN_Ki_MAX"));
            acceleration = reader.getDouble("ACCELERATION");

        } catch(Exception e){
            Log.e(" Drive Engine Error", "Config File Read Fail: " + e.toString());
            throw new RuntimeException("Drive Engine Config Read Failed!:" + e.toString());
        }
    }

    public void driveOnHeadingIMU(int heading, double desiredVelocity, LinearOpMode mode) {
       driveOnHeadingIMU(heading,desiredVelocity,10,mode);
    }

    public void driveOnHeadingIMU(int heading, double desiredVelocity, long delayTimeMillis, LinearOpMode mode) {
        desiredVelocity = Math.abs(desiredVelocity);
        double curOrientation = orientation.getOrientation();
        curOrientation += (heading - curOrientation);
        double distanceFromHeading = 0;
        distanceFromHeading = curOrientation - heading;
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        else if(distanceFromHeading < -180) distanceFromHeading += 360;
        headingController.setSp(0);
        double deltaVelocity = Math.abs(headingController.calculatePID(distanceFromHeading));
        //Log.d("heading", Double.toString(heading));
        //Log.d("Distance from heading", Double.toString(distanceFromHeading));
        //Log.d("Current heading", Double.toString(curOrientation));
        //Log.d("Delta velocity", Double.toString(deltaVelocity));
        //Log.d("Desired Velocity", Double.toString(desiredVelocity));
        //mode.telemetry.addData("Wanted heading", heading);
        //mode.telemetry.addData("Current heading", curOrientation);
        //mode.telemetry.addData("Dist from heading", distanceFromHeading);
//        if(heading - curHeading < 0) curHeading -= 360;
//        else if(heading - curHeading > 180) curHeading +=360;
        double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading,desiredVelocity);
        if(distanceFromHeading < 0){
            if(heading == NORTH) {
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            } else if(heading == SOUTH) {
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            } else if(heading == WEST){
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            } else if(heading == EAST){
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            }
        } else {
            if(heading == NORTH) {
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            } else if(heading == SOUTH){
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            } else if(heading == WEST){
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
            } else if(heading == EAST){
                velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaVelocity;
                velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
                velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity;
            }
        }
//        for(int i = 0; i < velocities.length; i ++){
//            Log.d("Velocity: " + i, "" + velocities[i] + "in/s");
//        }

        applyMotorVelocities(velocities);
        mode.sleep(delayTimeMillis);
    }

    public void newDriveOnHeadingIMU(int heading, double desiredVelocity, long delayTimeMillis, LinearOpMode mode) {
        desiredVelocity = Math.abs(desiredVelocity);
        double curOrientation = orientation.getOrientation();
        curOrientation += (heading - curOrientation);
        double distanceFromHeading = 0;
        distanceFromHeading = curOrientation - heading;
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        else if(distanceFromHeading < -180) distanceFromHeading += 360;
        headingController.setSp(0);
        double deltaVelocity = Math.abs(headingController.calculatePID(distanceFromHeading));
        //Log.d("heading", Double.toString(heading));
        //Log.d("Distance from heading", Double.toString(distanceFromHeading));
        //Log.d("Current heading", Double.toString(curOrientation));
        //Log.d("Delta velocity", Double.toString(deltaVelocity));
        //Log.d("Desired Velocity", Double.toString(desiredVelocity));
        //mode.telemetry.addData("Wanted heading", heading);
        //mode.telemetry.addData("Current heading", curOrientation);
        //mode.telemetry.addData("Dist from heading", distanceFromHeading);
//        if(heading - curHeading < 0) curHeading -= 360;
//        else if(heading - curHeading > 180) curHeading +=360;
        double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading,desiredVelocity);
        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += Math.sin(2*Math.toRadians(heading + 45));
        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += Math.sin(2*Math.toRadians(heading + 45));
        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += Math.sin(2*Math.toRadians(heading + 45));
        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += Math.sin(2*Math.toRadians(heading + 45));
//        for(int i = 0; i < velocities.length; i ++){
//            Log.d("Velocity: " + i, "" + velocities[i] + "in/s");
//        }

        applyMotorVelocities(velocities);
        mode.sleep(delayTimeMillis);
    }

    public void driveDistance(double distanceInInches, int heading, double desiredVelocity, LinearOpMode mode){
        //double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading, desiredVelocity);
        double distanceTraveled = 0;
        double [] motorPositionsInches = getMotorPositionsInches();
        double [] startPositionsInches = motorPositionsInches;
        //applyMotorVelocities(velocities);
        long startTime = System.currentTimeMillis();
        while(distanceTraveled < distanceInInches && mode.opModeIsActive()){
            //from our motor posisition, determine location
            driveOnHeadingIMU(heading,desiredVelocity,0,mode);
            motorPositionsInches = getMotorPositionsInches();
            double [] deltaInches = new double[4];
            for (int i = 0; i < motorPositionsInches.length; i ++){
                deltaInches[i] = Math.abs(motorPositionsInches[i] - startPositionsInches[i]);
            }
            double averagePosition = 0;
            for(double i : deltaInches){
                averagePosition += i;
            }
            averagePosition /= (double)deltaInches.length;
            distanceTraveled = averagePosition/Math.sin(Math.toRadians(45));

            //mode.telemetry.addData("Distance Travelled", distanceTraveled);
            //mode.telemetry.addData("Avg Position", averagePosition);
            //mode.telemetry.update();

            mode.sleep(50);
        }
        brake();
    }

    long [] getMotorPositionsTicks(){
        long [] positions = new long[4];
        positions[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        return  positions;
    }

    double [] getMotorPositionsInches(){
        double [] inches = new double [4];
        long [] ticks = getMotorPositionsTicks();
        inches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]));
        inches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]));
        inches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]));
        inches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].convertTicksToInches(ticks[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]));
        return inches;
    }

    double [] determineMotorVelocitiesToDriveOnHeading(double heading, double desiredVelocity) {
        double[] velocities = new double[4];
        //velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity*Math.cos()
        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.sin(Math.toRadians(heading + 45));
        velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.cos(Math.toRadians(heading + 45));
        velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.sin(Math.toRadians(heading + 45));
        velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredVelocity * Math.cos(Math.toRadians(heading + 45));
        return velocities;
    }

    public void driveOnHeading(double heading, double desiredVelocity) {
        applyMotorVelocities(determineMotorVelocitiesToDriveOnHeading(heading, desiredVelocity));
    }


    public void turn(double rps) {
        double[] velocities = new double[4];
        double velocity = rps*WHEEL_BASE_RADIUS*2.0*Math.PI;
        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = velocity;
        velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -velocity;
        velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = velocity;
        velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -velocity;
        applyMotorVelocities(velocities);
    }

    public double getDistanceFromHeading(double targetAngle){
        double distanceFromHeading = orientation.getOrientation() - targetAngle;
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        else if(distanceFromHeading < -180) distanceFromHeading += 360;
        return distanceFromHeading;
    }

    public void turnToHeading(double desiredHeading, LinearOpMode mode){
        headingController.setSp(0);
        double curHeading = orientation.getOrientation();
        double rps;
        double distanceFromHeading = 0;
        distanceFromHeading = orientation.getHeading() - desiredHeading;
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        else if(distanceFromHeading < -180) distanceFromHeading += 360;
        if(distanceFromHeading > 0){
            while(Math.abs(distanceFromHeading) > HEADING_THRESHOLD && mode.opModeIsActive()){

                //heading always positive
                Log.d("Distance From Heading","" + distanceFromHeading);
                //Log.d("Heading", Double.toString(curHeading));
                rps = turnController.calculatePID(distanceFromHeading);
                Log.d("RPS", Double.toString(rps));
                turn(-rps);
                //mode.sleep(10);
                curHeading = orientation.getOrientation();
                distanceFromHeading = curHeading - desiredHeading;
                if(distanceFromHeading > 180) distanceFromHeading -= 360;
                else if(distanceFromHeading < -180) distanceFromHeading += 360;
            }
            brake();
        }

        else if(distanceFromHeading < 0){
            while(Math.abs(distanceFromHeading) > HEADING_THRESHOLD && mode.opModeIsActive()){
                //heading always positive
                //Log.d("Heading", Double.toString(curHeading));
                Log.d("Distance From Heading","" + distanceFromHeading);
                rps = turnController.calculatePID(distanceFromHeading);
                Log.d("RPS", Double.toString(rps));
                turn(-rps);
                mode.sleep(10);
                curHeading = orientation.getOrientation();
                distanceFromHeading = curHeading - desiredHeading;
                if(distanceFromHeading > 180) distanceFromHeading -= 360;
                else if(distanceFromHeading < -180) distanceFromHeading += 360;
            }
            brake();
        }

    }

    public void setDrivePower(double power){
        double[] powers = new double[4];
        for(int i = 0; i < 4; i++){
            powers[i] = power;
        }
        applyMotorPowers(powers);
    }

    public void applyMotorVelocities(double [] velocities){
        driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
    }

    public void applyMotorPowers(double [] powers){
        driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
    }

    public void brake(){
        applyMotorVelocities(new double []{0,0,0,0});
    }

    public void stopNavigation(){
        shouldRun = false;
        for(int i =0; i < driveMotors.length; i ++){
            driveMotors[i].killMotorController();
        }
        orientation.stopIMU();
    }

    public static void driveToLocation(Location startLocation, Location targetLocation, double desiredSpeed, LinearOpMode mode){
        double distanceToTravel = startLocation.distanceToLocation(targetLocation);
        double deltaX = targetLocation.getX() - startLocation.getX();
        double deltaY = targetLocation.getY() - startLocation.getY();
        int heading = (int)Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90;
        heading = 360 - heading;
        if(heading >= 360) heading -= 360;
        if(heading < 0) heading += 360;
        Log.d("start x", Double.toString(startLocation.getX()));
        Log.d("start y", Double.toString(startLocation.getY()));
        Log.d("target x", Double.toString(targetLocation.getX()));
        Log.d("target y", Double.toString(targetLocation.getY()));
        Log.d("dist to travel", Double.toString(distanceToTravel));
        Log.d("heading", Double.toString(heading));

    }



    public HeadingVector[] getWheelVectors(){
        double [] deltaWheelPositions = {0,0,0,0};
        for(int i = 0; i < driveMotors.length; i ++){
            deltaWheelPositions[i] = driveMotors[i].getInchesFromStart() - lastMotorPositionsInInches[i];
        }
        HeadingVector [] wheelVectors = new HeadingVector[4];
        wheelVectors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FL_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        wheelVectors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FR_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        wheelVectors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BL_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        wheelVectors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BR_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        return wheelVectors;
    }
}
