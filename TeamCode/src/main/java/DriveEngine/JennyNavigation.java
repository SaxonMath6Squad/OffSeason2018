package DriveEngine;

import android.provider.CalendarContract;
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

/*
    The base class for every opmode --- it sets up our drive system and contains all it's funcitons
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
    public static final double ROBOT_WIDTH_INCHES = 18;
    public static final double ADJUSTING_SPEED_IN_PER_SEC = 5;
    public static final double SLOW_SPEED_IN_PER_SEC = 10;
    public static final double MED_SPEED_IN_PER_SEC = 20;
    public static final double DEFAULT_SPEED_IN_PER_SEC = 30;
    public static final double HIGH_SPEED_IN_PER_SEC = 50;
    public static final double LOCATION_DISTANCE_TOLERANCE = 0.5;
    public static final long DEFAULT_DELAY_MILLIS = 10;
    public static final long DEFAULT_SLEEP_DELAY_MILLIS = 50;
    public static final long MED_SLEEP_DELAY_MILLIS = 500;
    public static final long LONG_SLEEP_DELAY_MILLIS = 1000;
    private volatile long threadDelayMillis = 10;
    public volatile double robotHeading = 0;
    private volatile double [] lastMotorPositionsInInches = {0,0,0,0};
    private PIDController headingController, turnController, cameraPIDController;


    private volatile Location myLocation;
    private volatile HeadingVector [] wheelVectors = new HeadingVector[4];
    private volatile HeadingVector robotMovementVector = new HeadingVector();

    public ImuHandler orientation;
    private double orientationOffset = 0;
    private long timeAtAccelerationStart;
    private volatile boolean shouldRun = true;

    private final double HEADING_THRESHOLD = 3;
    private final double WHEEL_BASE_RADIUS = 20;
    private final double FL_WHEEL_HEADING_OFFSET = 45;
    private final double FR_WHEEL_HEADING_OFFSET = 315;
    private final double BR_WHEEL_HEADING_OFFSET = 45;
    private final double BL_WHEEL_HEADING_OFFSET = 315;
    private double acceleration = 0;
    private HardwareMap hardwareMap;
    public JennyNavigation(HardwareMap hw, Location startLocation, double robotOrientationOffset, String configFile) throws Exception{
        //initialize driveMotors
        //initialize sensors
        hardwareMap = hw;
        initializeUsingConfigFile(configFile);
        orientationOffset = robotOrientationOffset;
        orientation = new ImuHandler("imu", orientationOffset, hardwareMap);
        myLocation = new Location(startLocation.getX(),startLocation.getY());
        for(int i = 0; i < wheelVectors.length; i++){
            wheelVectors[i] = new HeadingVector();
        }
        for(int i = 0; i < lastMotorPositionsInInches.length; i ++){
            lastMotorPositionsInInches[i] = driveMotors[i].getInchesFromStart();
        }
        robotMovementVector = new HeadingVector();
        new Thread(new Runnable() {
            @Override
            public void run() {
                for (int i = 0; i < driveMotors.length; i++) {
                    Log.d("Inch from start", Integer.toString(i) + ": " + driveMotors[i].getInchesFromStart());
                }
                while (shouldRun) {
                    try {
                        updateData();
                    }
                    catch (Exception e){
                        shouldRun = false;
                        throw e;
                    }
                    safetySleep(threadDelayMillis);
                }
            }
        }).start();

    }

    private void updateLastMotorPositionsInInches(){
        for (int i = 0; i < driveMotors.length; i++){
            lastMotorPositionsInInches[i] = driveMotors[i].getInchesFromStart();
        }
    }

    private void updateHeading(){
        robotHeading = (orientation.getOrientation());
    }

    public Location getRobotLocation(){
        return new Location(myLocation.getX(),myLocation.getY());
    }

    private void updateLocation(){
        HeadingVector travelVector = wheelVectors[0].addVectors(wheelVectors);
        travelVector = new HeadingVector(travelVector.x()/2, travelVector.y()/2);
        Log.d("Wheel", "Dx: " + travelVector.x() + " Dy:" + travelVector.y());
        double headingOfRobot = travelVector.getHeading();
        Log.d("WHeel Heading", "" + headingOfRobot);
        Log.d("Orientation heading", "" + robotHeading);
        double magnitudeOfRobot = travelVector.getMagnitude();
        Log.d("Magnitude","" + magnitudeOfRobot);
        double actualHeading = (headingOfRobot + robotHeading)%360;

        Log.d("Robot Heading", "" + actualHeading);
        robotMovementVector.calculateVector(actualHeading, magnitudeOfRobot);
        double deltaX = robotMovementVector.x();
        double deltaY = robotMovementVector.y();
        myLocation.addXY(deltaX, deltaY);
        Log.d("Location","X:" + myLocation.getX() + " Y:" + myLocation.getY());

    }

    private void updateData(){
        updateHeading();
        wheelVectors = getWheelVectors();
        updateLocation();
        //updateLastMotorPositionsInInches();
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
            cameraPIDController = new PIDController(reader.getDouble("CAMERA_Kp"), reader.getDouble("CAMERA_Ki"), reader.getDouble("CAMERA_Kd"));
            cameraPIDController.setI(reader.getDouble("CAMERA_Ki_MAX"));
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

    public void newDriveOnHeadingIMU(double heading, double desiredVelocity, long delayTimeMillis, LinearOpMode mode) {
        desiredVelocity = Math.abs(desiredVelocity);
        double curOrientation = orientation.getOrientation();
        curOrientation += (heading - curOrientation);
        double distanceFromHeading = 0;
        distanceFromHeading = curOrientation - heading;
        if(distanceFromHeading > 180) distanceFromHeading -= 360;
        else if(distanceFromHeading < -180) distanceFromHeading += 360;
        headingController.setSp(0);
        double deltaVelocity = Math.abs(headingController.calculatePID(distanceFromHeading));

        /*
        okay, this is gonna be freaky....

        the idea behind this is to isolate the 'left' and 'right' side of the desired heading
        weight the wheels based upon their sin or cos as determined in the wheel vectoring
        add to the robot
         */
        int [] leftSideIndexes = {0,0,0,0};
        int [] rightSideIndexes = {0,0,0,0};
        if(heading > 315 || heading <= 45){
            leftSideIndexes[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
            leftSideIndexes[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
        }
        else if(heading > 45 && heading <= 135){
            leftSideIndexes[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
            leftSideIndexes[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
        }
        else if(heading > 135 && heading <= 225){
            leftSideIndexes[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
            leftSideIndexes[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
        }
        else if(heading > 225 && heading <= 315){
            leftSideIndexes[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
            leftSideIndexes[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = 1;
            rightSideIndexes[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = 1;
        }

        //weight the wheels appropriately
        double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading,desiredVelocity);
        for(int i = 0; i < driveMotors.length; i ++){
            if(leftSideIndexes[i] == 1) {
                if(distanceFromHeading > 0){
                    if(i%2 == 0){
                        velocities[i] += deltaVelocity*Math.sin(Math.toRadians(heading + 45));
                    }
                    else {
                        velocities[i] += deltaVelocity * Math.cos(Math.toRadians(heading + 45));
                    }
                }
                else if(distanceFromHeading < 0){
                    if(i%2 == 0){
                        velocities[i] -= deltaVelocity*Math.sin(Math.toRadians(heading + 45));
                    }
                    else {
                        velocities[i] -= deltaVelocity * Math.cos(Math.toRadians(heading + 45));
                    }
                }
            }
            if(rightSideIndexes[i] == 1){
                if(distanceFromHeading > 0){
                    if(i%2 == 0){
                        velocities[i] -= deltaVelocity*Math.sin(Math.toRadians(heading + 45));
                    }
                    else {
                        velocities[i] -= deltaVelocity * Math.cos(Math.toRadians(heading + 45));
                    }
                }
                else if(distanceFromHeading < 0){
                    if(i%2 == 0){
                        velocities[i] += deltaVelocity*Math.sin(Math.toRadians(heading + 45));
                    }
                    else {
                        velocities[i] += deltaVelocity * Math.cos(Math.toRadians(heading + 45));
                    }
                }
            }
        }

//        velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaVelocity * Math.sin(Math.toRadians(heading + 45));
//        velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += Math.cos(Math.toRadians(heading + 45));
//        velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += Math.sin(Math.toRadians(heading + 45));
//        velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += Math.cos(Math.toRadians(heading + 45));
//        for(int i = 0; i < velocities.length; i ++){
//            Log.d("Velocity: " + i, "" + velocities[i] + "in/s");
//        }

        applyMotorVelocities(velocities);
        mode.sleep(delayTimeMillis);
    }

    public void driveDistance(double distanceInInches, double heading, double desiredVelocity, LinearOpMode mode){
        //double [] velocities = determineMotorVelocitiesToDriveOnHeading(heading, desiredVelocity);
        distanceInInches = Math.abs(distanceInInches);
        double distanceTraveled = 0;
        double [] motorPositionsInches = getMotorPositionsInches();
        double [] startPositionsInches = motorPositionsInches;
        //applyMotorVelocities(velocities);
        //long deltaTime = 0;
        long startTime = 0;
        double [] deltaInches;
        double averagePosition = 0;
        while(distanceTraveled < distanceInInches && mode.opModeIsActive()){
            startTime = System.currentTimeMillis();
            //from our motor posisition, determine location
            newDriveOnHeadingIMU(heading,desiredVelocity,0,mode);
            motorPositionsInches = getMotorPositionsInches();
            deltaInches = new double[4];
            averagePosition = 0;
            if(heading == 45 || heading == 225){
                deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2;
                distanceTraveled = averagePosition;
            } else if(heading == 135 || heading == 315){
                deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
                deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = Math.abs(motorPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] - startPositionsInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
                averagePosition += deltaInches[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] + deltaInches[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR];
                averagePosition /= 2;
                distanceTraveled = averagePosition;
            } else {
                for (int i = 0; i < motorPositionsInches.length; i++) {
                    deltaInches[i] = Math.abs(motorPositionsInches[i] - startPositionsInches[i]);
                }
                for (double i : deltaInches) {
                    averagePosition += i;
                }
                averagePosition /= (double) deltaInches.length;
                distanceTraveled = averagePosition / Math.sin(Math.toRadians(45));
            }

            //mode.telemetry.addData("Distance Travelled", distanceTraveled);
            //mode.telemetry.addData("Avg Position", averagePosition);
            //mode.telemetry.update();
            //long loopTime = System.currentTimeMillis() - startTime;
            //Log.d("Loop time", Long.toString(loopTime) );
            //deltaTime = loopTime - deltaTime;
            //Log.d("Delta loop time", Long.toString(System.currentTimeMillis() - startTime));
//            mode.sleep(10);
        }
        brake();
        for (int i = 0; i < driveMotors.length; i++) {
            Log.d("Inch from start", Integer.toString(i) + ": " + driveMotors[i].getInchesFromStart());
        }
    }

    long [] getMotorPositionsTicks(){
        long [] positions = new long[4];
        positions[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        positions[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        return  positions;
    }

    public double [] getMotorPositionsInches(){
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

    private void driveToLocation(Location startLocation, Location targetLocation, double desiredSpeed, LinearOpMode mode){
        double distanceToTravel = startLocation.distanceToLocation(targetLocation);
        double deltaX;
        double deltaY;
        double heading;
        while(distanceToTravel > LOCATION_DISTANCE_TOLERANCE) {
            distanceToTravel = startLocation.distanceToLocation(targetLocation);
            deltaX = targetLocation.getX() - startLocation.getX();
            deltaY = targetLocation.getY() - startLocation.getY();
            heading = Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90;
            heading = 360 - heading;
            heading = (heading - orientation.getOrientation()) % 360;
            if (heading >= 360) heading -= 360;
            if (heading < 0) heading += 360;
            newDriveOnHeadingIMU(heading, desiredSpeed, 0, mode);
//            Log.d("start x", Double.toString(startLocation.getX()));
//            Log.d("start y", Double.toString(startLocation.getY()));
//            Log.d("target x", Double.toString(targetLocation.getX()));
//            Log.d("target y", Double.toString(targetLocation.getY()));
//            Log.d("dist to travel", Double.toString(distanceToTravel));
//            Log.d("heading", Double.toString(heading));
        }
    }

    public void driveToLocation(Location targetLocation, double desiredSpeed, LinearOpMode mode){
        driveToLocation(myLocation, targetLocation, desiredSpeed, mode);
    }

    public HeadingVector[] getWheelVectors(){
        double [] deltaWheelPositions = {0,0,0,0};
        for(int i = 0; i < driveMotors.length; i ++){
            double a = driveMotors[i].getInchesFromStart();
            deltaWheelPositions[i] = a - lastMotorPositionsInInches[i];
            lastMotorPositionsInInches[i] = a;
        }
        //updateLastMotorPositionsInInches();
        HeadingVector [] vectors = new HeadingVector[4];
        for(int i = 0; i < vectors.length; i++){
            vectors[i] = new HeadingVector();
        }
        vectors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FL_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FR_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BL_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BR_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        return vectors;
    }

    public void setLastMotorPositionsInInchesToOneInch(){
        for(int i = 0; i < lastMotorPositionsInInches.length; i ++){
            lastMotorPositionsInInches[i] = 1;
        }
    }

    public HeadingVector[] getWheelVectorsTest(){
        double [] deltaWheelPositions = {1,1,1,1};
        HeadingVector [] vectors = new HeadingVector[4];
        for(int i = 0; i < vectors.length; i++){
            vectors[i] = new HeadingVector();
        }
        vectors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FL_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(FR_WHEEL_HEADING_OFFSET,deltaWheelPositions[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BL_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        vectors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].calculateVector(BR_WHEEL_HEADING_OFFSET,deltaWheelPositions[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        for (int i = 0; i < vectors.length; i++){
            Log.d("Vector " + Integer.toString(i), "x: " + vectors[i].x() + ", y: " + vectors[i].y() + ", magnitude: " + vectors[i].getMagnitude());
        }
        return vectors;
    }

    public void driveToCryptobox(double cryptoBoxCenterX, double imageCenterX, double desiredSpeed, long delayTimeMillis, LinearOpMode mode){
        double velocities[] = new double[4];
        double deltaSpeed;
        double distanceFromCenter = imageCenterX - cryptoBoxCenterX;
        for(int i = 0; i < velocities.length; i++){
            velocities[i] = desiredSpeed;
        }
        cameraPIDController.setSp(0);
        deltaSpeed = Math.abs(distanceFromCenter);
        if(cryptoBoxCenterX > imageCenterX){
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaSpeed;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaSpeed;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] += deltaSpeed;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] -= deltaSpeed;
        }
        else {
            velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaSpeed;
            velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaSpeed;
            velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] -= deltaSpeed;
            velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] += deltaSpeed;
        }
        applyMotorVelocities(velocities);
        mode.sleep(delayTimeMillis);
    }

    public void driveToCryptobox(double cryptoBoxCenterX, double imageCenterX, double desiredSpeed, LinearOpMode mode){
        driveToCryptobox(cryptoBoxCenterX, imageCenterX, desiredSpeed, 10, mode);
    }
}
