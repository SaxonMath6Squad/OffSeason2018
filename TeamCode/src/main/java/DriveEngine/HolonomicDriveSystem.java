package DriveEngine;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.InputStream;

import Autonomous.HeadingVector;
import Autonomous.Location;
import MotorControllers.JsonConfigReader;
import MotorControllers.MotorController;
import MotorControllers.PIDController;
import SensorHandlers.ImuHandler;

/**
 * Created by robotics on 2/15/18.
 */

public class HolonomicDriveSystem {
    public MotorController[] driveMotors = new MotorController[4];
    public static final int FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR = 0;
    public static final int FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_HOLONOMIC_DRIVE_MOTOR = 3;
    private PIDController headingController, turnController;
    private volatile HeadingVector[] wheelVectors = new HeadingVector[4];
    private volatile HeadingVector robotMovementVector = new HeadingVector();
    public ImuHandler orientation;
    private double orientationOffset = 0;
    private volatile boolean shouldRun = true;
    private final double HEADING_THRESHOLD = 1;
    private final double WHEEL_BASE_RADIUS = 20;
    private final double FL_WHEEL_HEADING_OFFSET = 45;
    private final double FR_WHEEL_HEADING_OFFSET = 315;
    private final double BR_WHEEL_HEADING_OFFSET = 45;
    private final double BL_WHEEL_HEADING_OFFSET = 315;
    private HardwareMap hardwareMap;
    private Location robotLocation = new Location(0,0);

    private double maxMotorVelocity = 0;


    public HolonomicDriveSystem(HardwareMap hw, String configFile){
        hardwareMap = hw;
        readConfigAndInitialize(configFile);
    }

    public HolonomicDriveSystem(HardwareMap hw, Location startLocation, String configFile){
        this(hw, configFile);
        robotLocation = startLocation;
    }

    public HolonomicDriveSystem(HardwareMap hw, double robotOrientationOffset, Location startLocation, String configFile){
        this(hw, configFile);
        orientation.setOrientationOffset(robotOrientationOffset);
        robotLocation = startLocation;
    }

    public HolonomicDriveSystem(HardwareMap hw, double robotOrientationOffset, String configFile){
        this(hw, configFile);
        orientation.setOrientationOffset(robotOrientationOffset);
    }


    /**
     * reads the robot config file and initializes all motors using applicable data
     * @param file
     */
    public void readConfigAndInitialize(String file){
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
            driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("FRONT_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("FRONT_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("BACK_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("BACK_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            for (int i = 0; i < driveMotors.length; i++) {
                driveMotors[i].setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        } catch(Exception e){
            Log.e(" Drive Engine Error", "Config File Read Fail: " + e.toString());
            throw new RuntimeException("Drive Engine Config Read Failed!:" + e.toString());
        }
    }

    /**
     * normalizes all powers to -1 and 1, scales appropriately
     * @param toNormalize an array of doubles that have a wanted value of -1 to 1
     */
    private void normalizePowers(double [] toNormalize){
        //get the min and max powers
        double min = toNormalize[0], max = toNormalize[0];
        for(int i = 0; i < toNormalize.length; i ++){
            if(toNormalize[i] < min) min = toNormalize[i];
            else if(toNormalize[i] > max) max = toNormalize[i];
        }
        //assign toScaleAgainst to the largest (abs) value
        double toScaleAgainst = 0;
        if(Math.abs(min) < Math.abs(max)) toScaleAgainst = Math.abs(max);
        else toScaleAgainst = Math.abs(min);
        //if the largest (abs) is greater than 1, scale all values appropriately
        if(toScaleAgainst > 1){
            for(int i = 0; i < toNormalize.length; i ++){
                toNormalize[i] = toNormalize[i]/toScaleAgainst;
            }
        }
    }

    /**
     * calculates the powers required to make the robot move on a heading
     * @param heading from 0 to 360, represents the desired heading of the robot relative to the front of the robot
     * @param desiredPower from -1 to 1 that represents the desired proportion of max velocity for the robot to move at
     * @return a double array with the calculated motor powers for each wheel
     */
    private double [] calculatePowersToDriveOnHeading(double heading, double desiredPower){
        double[] powers = new double[4];
        powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.sin(Math.toRadians(heading + 45));
        powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.cos(Math.toRadians(heading + 45));
        powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.sin(Math.toRadians(heading + 45));
        powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.cos(Math.toRadians(heading + 45));
        return powers;
    }

    /**
     * calculates the powers of each motor to turn at the desired rate
     * @param desiredTurnRateOfMax a value from -1 to 1 that represents the rate of max turn to turn at
     * @return a double array with the calculated motor powers for each wheel
     */
    private double [] calculatePowersToTurn(double desiredTurnRateOfMax){
        double[] powers = new double[4];
        powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredTurnRateOfMax;
        powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -desiredTurnRateOfMax;
        powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredTurnRateOfMax;
        powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -desiredTurnRateOfMax;
        return powers;
    }

    /**
     * sets each motor at the desired motor power, -1 to 1 corresponding to max velocities
     * @param powers a double array of length 4 with values -1 to 1
     */
    private void applyMotorPowers(double [] powers){
        normalizePowers(powers);
        double [] velocities = new double[4];
        for(int i = 0; i < powers.length; i ++){
            velocities[i] = powers[i]*maxMotorVelocity;
        }
        applyMotorVelocities(velocities);
    }

    /**
     * sets each motor to turn at the desired velocity
     * @param velocities a double array of length 4 with the desired motor velocities to set the motors at
     */
    private void applyMotorVelocities(double [] velocities){
        driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
    }


    /**
     * brakes the robot
      */
    private void brake(){
        applyMotorPowers(new double[] {0,0,0,0});
    }

    /**
     * kills all parts of the robot for a safe shutdown
     */
    public void kill(){

    }
}
