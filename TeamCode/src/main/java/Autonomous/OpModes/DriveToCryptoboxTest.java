package Autonomous.OpModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Autonomous.RelicRecoveryField;
import Autonomous.Location;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.FAR_AWAY_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.FAR_AWAY_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.SCORING_COLUMN_1;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.MED_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.WEST;

/**
 * Created by robotics on 12/15/17.
 */

/*
    An opmode to test centering the robot on a cryptobox with the camera
 */
@Autonomous(name = "Center Test", group = "visual autonomous")
@Disabled
public class DriveToCryptoboxTest extends LinearOpMode{

    JennyNavigation nav;
    JennySensorTelemetry sensorTelemetry;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    VuforiaHelper vuforia;
    RelicRecoveryField field;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            nav = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT,DESIRED_WIDTH,FAR_AWAY_MIN_PERCENT_COLUMN_CHECK,FAR_AWAY_MIN_COLUMN_WIDTH);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            field = new RelicRecoveryField();
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Bitmap curImage = null;
        ArrayList<Integer> centers;
        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        if(curImage == null){
            throw new RuntimeException("Image capture failed!");
        }
        centers = cryptoBoxFinder.findColumnCenters(curImage, false);

        nav.driveToCryptobox(centers.get(0),cryptoBoxFinder.imageWidth/2, MED_SPEED_IN_PER_SEC, this);

        nav.stopNavigation();
        sensorTelemetry.stopSensorTelemetry();
    }


    public int determineDirectionToCryptoBox(int dirHint){
        return NORTH;
    }
    public int determineDirectionToCryptoBox(){
        return NORTH;
    }

    public boolean centerOnCryptoBox(int column, ArrayList<Integer> centers, int dirHint){
        if(cryptoBoxFinder.imageWidth/2 < centers.get(column).intValue()){
            if(cryptoBoxFinder.imageWidth/2  - centers.get(column).intValue() < centers.get(column).intValue()/10){
                nav.driveOnHeadingIMU(EAST, 5, this);
            } else {
                nav.brake();
                return true;
            }
        } else if(cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()){
            if(centers.get(column).intValue() - cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()/10){
                nav.driveOnHeadingIMU(WEST, 5, this);
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
