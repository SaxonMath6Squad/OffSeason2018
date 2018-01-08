package Autonomous.OpModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;

/**
 * Created by robotics on 12/15/17.
 */

/*
    An opmode to test how the camera takes images based on the rotation of the phone
 */
@Autonomous(name = "Image rotation test", group = "visual autonomous")
//@Disabled
public class ImageRotationTest extends LinearOpMode{

    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    VuforiaHelper vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT,DESIRED_WIDTH,.1,1);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Bitmap image = null;
        image = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        cryptoBoxFinder.findColumnCenters(image, true);
        vuforia.saveBMP(image);
    }
}
