package Autonomous.OpModes;

import android.graphics.Bitmap;
import android.net.wifi.WifiEnterpriseConfig;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import Autonomous.ImageAlignmentHelper;
import DriveEngine.JennyNavigation;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.WEST;

/**
 * Created by root on 11/20/17.
 */

/*
    An opmode to test saving images using vuforia
 */
@Autonomous(name="Camera Alignment Example", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class CameraAlignmentExample extends LinearOpMode{

    CryptoBoxColumnImageProcessor cryptoFinder;
    ImageAlignmentHelper cameraAligner;
    JennyNavigation navigation;

    @Override
    public void runOpMode() throws InterruptedException {
        int imageTaken = 0;
        //set our team's color to blue 
        CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE;
        //initialize our drive system 
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
        } catch (Exception e){
            Log.e("Error!", "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Cration Error! " + e.toString());
        }
        //initalize VuforiaHelper
        VuforiaHelper vuforia = new VuforiaHelper();
        //initalize the camera alignment 
        cameraAligner = new ImageAlignmentHelper(DESIRED_WIDTH, navigation, this);

        //wait for the op mode to start, this is the time to change teams
        while (!opModeIsActive()) {
            //change the team's color until the start button is pressed 
            if (gamepad1.start) {
                if(color == CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE) color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.RED;
                else color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE;
                while (gamepad1.start) ;
            }
            telemetry.addData("Color", color);
            telemetry.update();
        }
        //storage variables for the alignment 
        Bitmap bmp;
        ArrayList<Integer> columnLocations = new ArrayList<Integer>();
        //initalize the image processor
        cryptoFinder = new CryptoBoxColumnImageProcessor(CryptoBoxColumnImageProcessor.DESIRED_HEIGHT, DESIRED_WIDTH,.1,1, color);
        telemetry.addData("Status","Initialized");

        waitForStart();

        telemetry.addData("Centering...", "");
        telemetry.update();
        // take an image with the correct size 
        bmp = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        //find the location of the colored columns, do not modify the image 
        columnLocations = cryptoFinder.findColumns(bmp, false);
        //until aligned, keep trying to align 
        while (!cameraAligner.centerOnCryptoBoxClosestToCenter(0, columnLocations, WEST, EAST) && opModeIsActive()) {
            // keep updating your image and column locations until you are centered
            bmp = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            columnLocations = cryptoFinder.findColumns(bmp, false);
        }
        telemetry.addData("Centered!", "");
        telemetry.update();
    }
}
