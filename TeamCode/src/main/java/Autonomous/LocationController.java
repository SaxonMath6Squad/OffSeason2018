package Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Autonomous.GeoLocator;
import Autonomous.Location;

import static Autonomous.RelicRecoveryField.*;

/**
 * Created by robotics on 12/21/17.
 */

/*
    A class to control robot location more easily
 */
public class LocationController implements GeoLocator{
    Location currentLocation;
    Location baseLocation;
    double rotation;

    public LocationController(Location startLocation, double rotationInDeg){
        baseLocation = startLocation;
        currentLocation = startLocation;
        rotation = rotationInDeg;
    }

    public double distanceToLocation(Location targetLocation){
        return currentLocation.distanceToLocation(targetLocation);
    }

    public double getRotation(){
        return rotation;
    }

    @Override
    public Location getLocation(){
        return currentLocation;
    }

    @Override
    public void setBaseLocation(Location location){
        baseLocation = location;
    }

    private void updateLocation(double[] motorPositionInInches){


    }
}
