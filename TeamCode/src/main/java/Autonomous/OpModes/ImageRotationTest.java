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
import Systems.JennyO1BPickAndExtend;

import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.WEST;

/**
 * Created by robotics on 12/15/17.
 */

/*
    An opmode to test how the camera takes images based on the rotation of the phone
 */
@Autonomous(name = "Image rotation test", group = "visual autonomous")
@Disabled
public class ImageRotationTest extends LinearOpMode{

    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    VuforiaHelper vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(80,100,.1,1);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Bitmap image = null;
        image = vuforia.getImage(171, 244);
        cryptoBoxFinder.findColumnCenters(image, true);
        vuforia.saveBMP(image);
    }
}
