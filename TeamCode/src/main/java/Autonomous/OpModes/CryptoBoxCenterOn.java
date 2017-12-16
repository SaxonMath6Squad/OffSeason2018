package Autonomous.OpModes;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.reflect.Array;
import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import DriveEngine.JennyNavigation;
import Systems.JennyV2PickAndExtend;
import Autonomous.VuforiaHelper;

import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SOUTH;
import static DriveEngine.JennyNavigation.WEST;

/**
 * Created by robotics on 12/15/17.
 */

@Autonomous(name = "cryptoBox Column Aligner", group = "visual autonomous")
public class CryptoBoxCenterOn extends LinearOpMode{

    JennyNavigation nav;
    JennyV2PickAndExtend glyphSystem;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    VuforiaHelper vuforia;


    @Override
    public void runOpMode() throws InterruptedException {
        try{
            nav = new JennyNavigation(hardwareMap, "RobotConfig/JennyV2.json");
            glyphSystem = new JennyV2PickAndExtend(hardwareMap);
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(80,100,.1,1);
        } catch (Exception e){
            throw new RuntimeException(e);
        }

        waitForStart();
        nav.driveDistance(2*12,WEST,30,this);
        nav.brake();
        Bitmap curImage = null;
        ArrayList<Integer> centers;
        curImage = vuforia.getImage();
        if(curImage == null){
            //implement a try here for a couple seconds...
            throw new RuntimeException("image captured failed...");
        }

        centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        while(centers.size() == 0 && opModeIsActive()){
            nav.driveOnHeadingIMU(WEST, 5, this);
            curImage = vuforia.getImage();
            centers = cryptoBoxFinder.findColumnCenters(curImage,false);
        }
        nav.brake();

        long startMillis = System.currentTimeMillis();
        while(centerOnCryptoBox(0, centers, JennyNavigation.WEST) == false && opModeIsActive()){
            long timeCaptureStart = System.currentTimeMillis();
            curImage = vuforia.getImage();
            long algorithmStart = System.currentTimeMillis();
            centers = cryptoBoxFinder.findColumnCenters(curImage,false);
            telemetry.addData("WantedColumnLoc","" + centers.get(0).intValue());
            telemetry.addData("CurLoc", "" + cryptoBoxFinder.imageWidth/2);
            telemetry.addData("Total Time", "" + (System.currentTimeMillis() - startMillis));
            //telemetry.addData("Image Capture Time", "" + (algorithmStart - timeCaptureStart));
            //telemetry.addData("Algorithm Time","" + (System.currentTimeMillis() - algorithmStart));
            telemetry.update();
            startMillis = System.currentTimeMillis();
        }
        nav.brake();
        nav.driveDistance(5,NORTH,20,this);
        nav.brake();
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
                nav.driveOnHeadingIMU(WEST, 5, this);
            } else {
                nav.brake();
                return true;
            }
        } else if(cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()){
            if(centers.get(column) - cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()/10){
                nav.driveOnHeadingIMU(EAST, 5, this);
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
