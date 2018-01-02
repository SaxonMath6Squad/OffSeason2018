package Autonomous.OpModes;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import Autonomous.VuforiaHelper;
import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;

/**
 * Created by root on 11/20/17.
 */

/*
    An opmode to test saving images using vuforia
 */
@Autonomous(name="Save Image Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class VuforiaImageCaptureTest extends LinearOpMode{

    CryptoBoxColumnImageProcessor cryptoFinder;
    final int IMAGE_PROCESS_WIDTH = 171;
    final int IMAGE_PROCESS_HEIGHT = 224;
    @Override
    public void runOpMode() throws InterruptedException {

        /*To access the image: you need to iterate through the images of the frame object:*/
        VuforiaHelper vuforia = new VuforiaHelper();
        cryptoFinder = new CryptoBoxColumnImageProcessor(100,120,.1,1);
        telemetry.addData("Status","Initialized");
        telemetry.update();
        waitForStart();
        Bitmap bmp;
        ArrayList<Integer> columnLocations = new ArrayList<Integer>();
        while(opModeIsActive()){
            long timeStart = System.currentTimeMillis();
            bmp = vuforia.getImage(120,100);
            if(bmp != null){
                //vuforia.saveBMP(bmp);
                long algorithmStart = System.currentTimeMillis();
                columnLocations = cryptoFinder.findColumns(bmp,false);
                telemetry.addData("Algorithm Time", "" + (System.currentTimeMillis() - algorithmStart));
                if(columnLocations != null){
                    for(int i = 0; i < columnLocations.size(); i ++){
                        telemetry.addData("Column " + i, " " + columnLocations.get(i).intValue());
                    }
                }
                telemetry.addData("Loop Time", "" + (System.currentTimeMillis() - timeStart));
                telemetry.update();
            }
            else{
                Log.d("BMP","NULL!");
            }
        }

    }







}
