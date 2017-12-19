package SensorHandlers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import MotorControllers.JsonConfigReader;
import Autonomous.Location;

//import static Hardware.JennyWithExtendotronHardware.*;

/**
 * Created by Jeremy on 8/26/2017.
 */

public class JennySensorTelemetry implements RobotSensorTelemetry {
//    private JennyWithExtendotronHardware robot;
    private JsonConfigReader reader;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private DigitalChannel[] limitSwitches = new DigitalChannel[4];
    private ImuHandler imu;
    private boolean shouldRun;
    private long loopTime = 100;
    private long lastLoopTime = 0;
    private HardwareMap hardwareMap;
    private int tickLocationX = 0;
    private int tickLocationY = 0;
    private int startPositionX = 0;
    private int startPositionY = 0;
    private double wheelDiameter;
    private double ticksPerRev;
    private double ftToTicksFactor = 12;
    public final int EXTEND_LIMIT = 0;
    public JennySensorTelemetry(HardwareMap h, int positionX, int positionY){
        hardwareMap = h;
//        robot = new JennyWithExtendotronHardware(hardwareMap);
        startPositionX = positionX;
        startPositionY = positionY;
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        limitSwitches[EXTEND_LIMIT] = hardwareMap.digitalChannel.get("extendLimit");
        try {
            reader = new JsonConfigReader(h.appContext.getAssets().open("MotorConfig/DriveMotors/HolonomicDriveMotorConfig.json"));
        } catch (Exception e){
            Log.d("Error: ", e.toString());
        }
        try {
            wheelDiameter = reader.getDouble("WHEEL_DIAMETER");
        } catch (Exception e){
            Log.d("Error: ", e.toString());
        }
        try {
            ticksPerRev = reader.getDouble("TICKS_PER_REV");
        } catch (Exception e){
            Log.d("Error: ", e.toString());
        }
    }
    @Override
    public Location getLocation() {
        Location location = new Location(0, 0);
//        tickLocationX = robot.driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentPositionInTicks();
//        tickLocationY = robot.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentPositionInTicks();
        location.updateXY(tickLocationX + startPositionX*ticksPerRev, tickLocationY + startPositionY*ticksPerRev);
        return location;
    }

    @Override
    public void setBaseLocation(Location location) {

    }

    @Override
    public double getHeading() {
        return 0;
    }

    @Override
    public double setBaseOrientation(double degree) {
        return 0;
    }

    @Override
    public double getOrientation() {
        return 0;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public void startTelemetryLogging() {

    }

    @Override
    public void stopTelemetryLogging() {

    }

    @Override
    public void resetTelemetryLogging() {

    }

    @Override
    public double getDistance(DistanceUnit unit){
        double distance = 0;
        double curDist = distanceSensor.getDistance(unit);
        if(curDist > 0 && curDist < 40 && !Double.isNaN(curDist)){
            distance = curDist;
        }
        return distance;
    }

    @Override
    public boolean getState(int sensor){
        return limitSwitches[sensor].getState();
    }
}
