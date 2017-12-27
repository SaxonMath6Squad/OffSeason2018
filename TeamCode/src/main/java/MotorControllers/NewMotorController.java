//use the standard DC Motor Controller but create wrapper for it to more easily control velocities, distances, etc.

package MotorControllers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;
import java.io.InputStream;

/**
 * Created by root on 11/11/17.
 */

public class NewMotorController extends Thread {
    //config file values
    private long maxTicksPerSecond = 0;
    private long ticksPerRevolution = 0;
    private double wheelDiameterInInches = 0;
    private double percentPerTickPerSec;
    private double powerOffset;


    //user set and program updated variables
    private volatile long desiredTicksPerSecVelocity = 0;
    private long currentTicksPerSec = 0;
    private long startTick = 0;
    private long endPosition = 0;
    private long ticksToTravel; // reflect + and -
    private long curTickLocation = 0;
    DcMotor motor;
    private String logTag = "";
    private boolean shouldLog = false;
    private volatile boolean shouldRun = false;
    MotorTachometer tachometer;

    private long LOOP_MILLIS = 200;

    HardwareMap hardwareMap;

    public NewMotorController(DcMotor m, String configFileLoc, HardwareMap hw)throws IOException {
        hardwareMap = hw;
        motor = m;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //read the config file. If error received, fail entire initialization
        if(readConfig(configFileLoc) != 1){
            Log.e(logTag + " MotorController Error","Config File Read Failed! Killing self!");
            shouldRun = false;
            throw new IOException("Failed to parse Motor Config File: " + configFileLoc);
        }
        tachometer = new MotorTachometer(m,ticksPerRevolution, MotorTachometer.RPS_SMOOTHER.NONE);
        shouldRun = true;
        Log.d("Ticks per rev", Double.toString(ticksPerRevolution));
        new Thread(new Runnable() {
            @Override
            public void run() {
                long loopStartMillis;
                while(shouldRun) {
                    //get start time
                    loopStartMillis = System.currentTimeMillis();
                    //update for runtime
                    updateData();
                    long remainingTime = LOOP_MILLIS - (System.currentTimeMillis() - loopStartMillis);
                    if (remainingTime > 0) safetySleep(remainingTime);
                }
            }
        });
    }

    public NewMotorController(String motorName, String configFileLoc, HardwareMap hw) throws IOException{
        this(hw.dcMotor.get(motorName), configFileLoc, hw);
    }

    public DcMotor.RunMode getMotorMode(){
        return motor.getMode();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b){
        motor.setZeroPowerBehavior(b);
    }

    public void setMotorDirection(DcMotor.Direction dir){
        motor.setDirection(dir);
    }

    public void setMotorMode(DcMotor.RunMode mode){
        motor.setMode(mode);
    }

    private void safetySleep(long time){
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < time && shouldRun);
    }

    public void killMotorController(){
        shouldRun = false;
        brake();
    }

    private void updateData(){
        //get new RPS
        double currentRPS = 0;
        try {
            currentRPS = tachometer.getRotationsPerSecond();
        } catch(Exception e){
            Log.e("NewMotorController Err",e.toString());
            shouldRun = false;
            throw new RuntimeException(e);
        }
        currentTicksPerSec = (long)(currentRPS * ticksPerRevolution + .5);
        //update position
        curTickLocation = motor.getCurrentPosition();
    }

    private int readConfig(String fileLoc){
        InputStream stream = null;
        try {
            stream = hardwareMap.appContext.getAssets().open(fileLoc);
        }
        catch(Exception e){
            Log.d("Error: ",e.toString());
        }
        JsonConfigReader reader = new JsonConfigReader(stream);
        try{
            //ticksPerRevolution = reader.getLong("TICKS_PER_REV");
            ticksPerRevolution = (long)motor.getMotorType().getTicksPerRev();
            wheelDiameterInInches = reader.getDouble("WHEEL_DIAMETER");
            //double maxRPS = reader.getDouble("MAX_RPS");
            double maxRPS = motor.getMotorType().getMaxRPM();
            //maxTicksPerSecond = (long)(maxRPS * ticksPerRevolution + .5);
            maxTicksPerSecond = (long) motor.getMotorType().getAchieveableMaxTicksPerSecondRounded();
        } catch(Exception e){
            Log.e(logTag + " MotorController Error", "Config File Read Fail: " + e.toString());
            return 0;
        }
        return 1;
    }

    public double getMotorPower(){
        return motor.getPower();
    }

    public long getCurrentTicksPerSecond(){
        return currentTicksPerSec;
    }

    public double getCurrentInchesPerSecond(){
        return (double)getCurrentTicksPerSecond() / (double)(ticksPerRevolution) * wheelDiameterInInches * Math.PI;
    }

    public double getCurrentRPS(){
        return (double)getCurrentTicksPerSecond() / (double)ticksPerRevolution;
    }

    public long getCurrentTick(){
        return motor.getCurrentPosition();
    }

    public double getInchesFromStart(){
        return (double) getCurrentTick() / (double)(ticksPerRevolution) * wheelDiameterInInches * Math.PI;
    }

    public void setTicksPerSecondVelocity(long ticksPerSec){
        //check for flip in sign, if so, reset our pid controller
        //power is a function of ticksPerSec/maxAcheivable ticks per second
        Log.d("MotorVel", "" + ticksPerSec + " T/s");
        try {
            motor.setPower((double) ticksPerSec / motor.getMotorType().getAchieveableMaxTicksPerSecondRounded());
        } catch (Exception e){
            Log.e("Motorcontroller Error", "SetTicksPerSecondVelocity: " + e.toString());
            shouldRun = false;
            throw e;
        }
        Log.d("MotorPow", "" + getMotorPower() + " %");

    }

    public void setInchesPerSecondVelocity(double inchesPerSec){
        long ticksPerSec = (long)(inchesPerSec/(wheelDiameterInInches*Math.PI)*ticksPerRevolution + .5);
        setTicksPerSecondVelocity(ticksPerSec);
    }

    public double convertTicksToInches(long ticks){
        return (double) ticks / ticksPerRevolution * Math.PI  * wheelDiameterInInches;
    }


    public void setMotorPower(double power){
        motor.setPower(power);
    }

    public void brake(){
        motor.setPower(0);
    }

    public void setPosition(double positionInInches){
        int positionInTicks = (int)(positionInInches/(wheelDiameterInInches*Math.PI)*ticksPerRevolution);
        Log.d("Desired tick", Double.toString(positionInTicks));
        motor.setTargetPosition(positionInTicks);
    }
}