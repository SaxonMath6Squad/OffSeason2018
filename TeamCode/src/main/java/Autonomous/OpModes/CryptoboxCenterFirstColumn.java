package Autonomous.OpModes;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;
import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Systems.JennyO1BPickAndExtend;

import Autonomous.VuforiaHelper;

import static Autonomous.RelicRecoveryField.*;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SOUTH;
import static DriveEngine.JennyNavigation.WEST;

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
    JennyO1BPickAndExtend glyphSystem;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    VuforiaHelper vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            nav = new JennyNavigation(hardwareMap, "RobotConfig/JennyV2.json");
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(80,100,.1,1);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            glyphSystem = new JennyO1BPickAndExtend(hardwareMap);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        nav.driveDistance(RIGHT_COLUMN_DISTANCE_TO_STONE_INCHES,EAST,30,this);
        Bitmap curImage = null;
        ArrayList<Integer> centers;
        curImage = vuforia.getImage(171,244);
        if(curImage == null){
            //implement a try here for a couple seconds...
            throw new RuntimeException("image captured failed...");
        }

        centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        while(centers.size() == 0 && opModeIsActive()){
            nav.driveOnHeadingIMU(EAST, 5, this);
            curImage = vuforia.getImage(171,244);
            centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        }
        nav.brake();

        while(centerOnCryptoBox(0, centers, JennyNavigation.EAST) == false && opModeIsActive()){
            curImage = vuforia.getImage(171,244);
            centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        }
        nav.brake();
        double distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        long timeLimit = CRYPTOBOX_APPROACH_TIME_LIMIT;
        while (distToWall == -1 && currentTime - startTime < timeLimit && opModeIsActive()){
            nav.driveOnHeadingIMU(SOUTH, 5, this);
            distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
            Log.d("Dist to wall", Double.toString(distToWall));
            currentTime = System.currentTimeMillis();
        }
        nav.brake();
        glyphSystem.liftToPosition(6);
        sleep(500);
        glyphSystem.setBeltPower(0.5);
        sleep(500);
        glyphSystem.pauseBelt();
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
