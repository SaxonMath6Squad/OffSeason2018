package SensorHandlers;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Autonomous.Location;

/**
 * Created by Jeremy on 8/26/2017.
 */

public interface RobotSensorTelemetry {
    public Location getLocation();
    public void setBaseLocation(Location location);
    public double getHeading();
    public double setBaseOrientation(double degree);
    public double getOrientation();
    public double getVelocity();
    public void startTelemetryLogging();
    public void stopTelemetryLogging();
    public void resetTelemetryLogging();
    public double getDistance(DistanceUnit unit);
    public boolean isPressed(int sensor);
}
