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

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.WANTED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.WANTED_WIDTH;
import static Autonomous.RelicRecoveryField.*;
import static DriveEngine.JennyNavigation.ADJUSTING_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.DEFAULT_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.HIGH_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.MED_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SOUTH;
import static DriveEngine.JennyNavigation.WEST;
import static SensorHandlers.JennySensorTelemetry.NO_DETECTABLE_WALL_DISTANCE;
import static SensorHandlers.JennySensorTelemetry.START_LOCATION_X;
import static SensorHandlers.JennySensorTelemetry.START_LOCATION_Y;

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
            nav = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2],90,"RobotConfig/JennyV2.json");
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT,DESIRED_WIDTH,PERCENT_COLUMN_CHECK,MIN_COLUMN_WIDTH);
            field = new RelicRecoveryField();
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, startLocations[BLUE_ALLIANCE_2].getX(), startLocations[BLUE_ALLIANCE_2].getY());
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        nav.driveToLocation(field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1), HIGH_SPEED_IN_PER_SEC, this);
        Log.d("Cryptobox Location", "X: " + field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1).getX() + ", Y: " + field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1).getY());
        Bitmap curImage = null;
        ArrayList<Integer> centers;
        curImage = vuforia.getImage(WANTED_WIDTH,WANTED_HEIGHT);
        if(curImage == null){
            //implement a try here for a couple seconds...
            throw new RuntimeException("image captured failed...");
        }

        centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        while(centers.size() == 0 && opModeIsActive()){
            nav.newDriveOnHeadingIMU(EAST, ADJUSTING_SPEED_IN_PER_SEC, DEFAULT_DELAY_MILLIS, this);
            curImage = vuforia.getImage(WANTED_WIDTH,WANTED_HEIGHT);
            centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        }
        nav.brake();

        while(centerOnCryptoBox(0, centers, JennyNavigation.EAST) == false && opModeIsActive()){
            curImage = vuforia.getImage(WANTED_WIDTH,WANTED_HEIGHT);
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

        nav.driveToLocation(field.determinePitLocation(BLUE_ALLIANCE_2), MED_SPEED_IN_PER_SEC, this);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        nav.driveToLocation(field.determineLocationOfCryptoboxScoringCenter(BLUE_ALLIANCE_2, SCORING_COLUMN_1), MED_SPEED_IN_PER_SEC, this);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
    }

    public boolean centerOnCryptoBox(int column, ArrayList<Integer> centers, int dirHint){
        if(cryptoBoxFinder.imageWidth/2 < centers.get(column).intValue()){
            if(cryptoBoxFinder.imageWidth/2  - centers.get(column).intValue() < centers.get(column).intValue()/10){
                nav.newDriveOnHeadingIMU(EAST, ADJUSTING_SPEED_IN_PER_SEC, DEFAULT_DELAY_MILLIS, this);
            } else {
                nav.brake();
                return true;
            }
        } else if(cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()){
            if(centers.get(column).intValue() - cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()/10){
                nav.newDriveOnHeadingIMU(WEST, ADJUSTING_SPEED_IN_PER_SEC, DEFAULT_DELAY_MILLIS, this);
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
