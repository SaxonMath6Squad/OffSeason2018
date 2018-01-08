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
import Actions.JennyO1BGlyphExtender;

import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.EAST;
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
public class CenterTest extends LinearOpMode{

    JennyNavigation nav;
    JennySensorTelemetry sensorTelemetry;
    JennyO1BGlyphExtender glyphSystem;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    VuforiaHelper vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            nav = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(80,100,.1,1);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            glyphSystem = new JennyO1BGlyphExtender(hardwareMap);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Bitmap curImage = null;
        ArrayList<Integer> centers;
        curImage = vuforia.getImage(171,244);
        if(curImage == null){
            //implement a try here for a couple seconds...
            throw new RuntimeException("image captured failed...");
        }

        centers = cryptoBoxFinder.findColumnCenters(curImage,true);
        while(centers.size() == 0 && opModeIsActive()){
            nav.driveOnHeadingIMU(EAST, 5, this);
            curImage = vuforia.getImage(171,244);
            centers = cryptoBoxFinder.findColumnCenters(curImage,true);
            vuforia.saveBMP(curImage);
        }
        nav.brake();

        while(centerOnCryptoBox(0, centers, JennyNavigation.EAST) == false && opModeIsActive()){
            curImage = vuforia.getImage(171,244);
            centers = cryptoBoxFinder.findColumnCenters(curImage,true);
            vuforia.saveBMP(curImage);
        }
        nav.brake();

        nav.stopNavigation();
        sensorTelemetry.stopTelemetryLogging();
        glyphSystem.stop();
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
