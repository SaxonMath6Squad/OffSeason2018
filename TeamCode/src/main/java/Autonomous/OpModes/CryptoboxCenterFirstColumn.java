package Autonomous.OpModes;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Autonomous.RelicRecoveryField;

import Autonomous.VuforiaHelper;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.*;
import static Autonomous.RelicRecoveryField.*;
import static DriveEngine.JennyNavigation.*;
import static SensorHandlers.JennySensorTelemetry.*;
import Autonomous.Location;

/**
 * Created by robotics on 12/15/17.
 */

/*
    An opmode to test centering on the first column of a cryptobox that the camera finds and then score a glyph in it
 */
@Autonomous(name = "Cryptobox Center First Column", group = "visual autonomous")
//@Disabled
public class CryptoboxCenterFirstColumn extends LinearOpMode{

    JennyNavigation nav;
    JennySensorTelemetry sensorTelemetry;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    VuforiaHelper vuforia;
    RelicRecoveryField field;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            nav = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2],270,"RobotConfig/JennyV2.json");
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT,DESIRED_WIDTH, CLOSE_UP_MIN_PERCENT_COLUMN_CHECK, CLOSE_UP_MIN_COLUMN_WIDTH);
            field = new RelicRecoveryField();
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, startLocations[BLUE_ALLIANCE_2].getX(), startLocations[BLUE_ALLIANCE_2].getY());
        } catch (Exception e){
            throw new RuntimeException(e);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Location startTargetCryptoboxLocation = new Location(nav.getRobotLocation().getX(), field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1).getY());
        nav.driveToLocation(startTargetCryptoboxLocation, MED_SPEED_IN_PER_SEC, this);
        Log.d("Cryptobox Location", "X: " + field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1).getX() + ", Y: " + field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1).getY());
        Bitmap curImage = null;
        ArrayList<Integer> centers;
        curImage = vuforia.getImage(DESIRED_WIDTH,DESIRED_HEIGHT);
        if(curImage == null){
            //implement a try here for a couple seconds...
            throw new RuntimeException("image captured failed...");
        }

        centers = cryptoBoxFinder.findColumnCenters(curImage,false);
//        while(centers.size() == 0 && opModeIsActive()){
//            nav.newDriveOnHeadingIMU(WEST, ADJUSTING_SPEED_IN_PER_SEC, DEFAULT_DELAY_MILLIS, this);
//            curImage = vuforia.getImage(DESIRED_WIDTH,DESIRED_HEIGHT);
//            centers = cryptoBoxFinder.findColumnCenters(curImage,false);
//        }
//        nav.brake();

        while(centerOnCryptoBox(0, centers, WEST) == false && opModeIsActive()){
            curImage = vuforia.getImage(DESIRED_WIDTH,DESIRED_HEIGHT);
            centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        }
        nav.brake();

        double distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        long timeLimit = CRYPTOBOX_APPROACH_TIME_LIMIT;
        while (distToWall == NO_DETECTABLE_WALL_DISTANCE && currentTime - startTime < timeLimit && opModeIsActive()){
            nav.driveOnHeadingIMU(SOUTH, ADJUSTING_SPEED_IN_PER_SEC, this);
            distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
            Log.d("Dist to wall", Double.toString(distToWall));
            currentTime = System.currentTimeMillis();
        }
        nav.brake();

        sleep(LONG_SLEEP_DELAY_MILLIS);
        Location targetPitLocation = new Location(field.determinePitLocation(BLUE_ALLIANCE_2, nav.getRobotLocation()).getX(), nav.getRobotLocation().getY());
        Log.d("Target Pit Location: ", "X: " + targetPitLocation.getX() + ", Y: " + targetPitLocation.getY());
        nav.driveToLocation(targetPitLocation, MED_SPEED_IN_PER_SEC, this);
        sleep(MED_SLEEP_DELAY_MILLIS);

        Location endCryptoboxLocation = new Location(field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1).getX() - CRYPTOBOX_THICKNESS_INCHES*1.5 - ROBOT_WIDTH_INCHES/2, nav.getRobotLocation().getY());
        nav.driveToLocation(endCryptoboxLocation, MED_SPEED_IN_PER_SEC, this);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);

        ArrayList<Integer> newCenters;
        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        if(curImage == null){
            throw new RuntimeException("Image capture failed!");
        }
        newCenters = cryptoBoxFinder.findColumnCenters(curImage, false);
        while (centerOnCryptoBox(0, newCenters, EAST) == false && opModeIsActive()){
            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            newCenters = cryptoBoxFinder.findColumnCenters(curImage, false);
        }
        nav.brake();

//        distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
//        startTime = System.currentTimeMillis();
//        currentTime = System.currentTimeMillis();
//        timeLimit = CRYPTOBOX_APPROACH_TIME_LIMIT;
//        while (distToWall == NO_DETECTABLE_WALL_DISTANCE && currentTime - startTime < timeLimit && opModeIsActive()){
//            nav.driveOnHeadingIMU(SOUTH, ADJUSTING_SPEED_IN_PER_SEC, this);
//            distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
//            Log.d("Dist to wall", Double.toString(distToWall));
//            currentTime = System.currentTimeMillis();
//        }
//        nav.brake();
//
        nav.stopNavigation();
    }

    public boolean centerOnCryptoBox(int column, ArrayList<Integer> centers, int dirHint){
        //Log.d("")
        if(centers.size() == 0){
            nav.newDriveOnHeadingIMU(dirHint, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
            return false;
        }
        if(cryptoBoxFinder.imageWidth/2 < centers.get(column).intValue()){
            if(Math.abs(cryptoBoxFinder.imageWidth/2  - centers.get(column).intValue()) < cryptoBoxFinder.imageWidth/10){
                nav.newDriveOnHeadingIMU(WEST, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
            } else {
                nav.brake();
                return true;
            }
        } else if(cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()){
            if(Math.abs(centers.get(column).intValue() - cryptoBoxFinder.imageWidth/2)  > cryptoBoxFinder.imageWidth/10){
                nav.newDriveOnHeadingIMU(EAST, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
            } else {
                nav.brake();
                return true;
            }
        } else {
            nav.brake();
            return true;
        }
        return false;
    }
}
