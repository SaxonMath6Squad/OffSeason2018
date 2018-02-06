package Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import DriveEngine.JennyNavigation;
import MotorControllers.PIDController;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static DriveEngine.JennyNavigation.SLOW_SPEED_IN_PER_SEC;

/**
 * Created by robotics on 1/27/18.
 */

public class ImageAlignmentHelper {
    private int CRYPTO_COLUMN_TARGET_POSITION = 40;
    private double MOVEMENT_KP_UNSCALED = 10;
    private double MOVEMENT_KI = 0;
    private double MOVEMENT_KI_MAX = 0;
    private int DESIRED_PIXEL_ACCURACY = 3;
    private PIDController imagePid;
    private LinearOpMode mode;
    private JennyNavigation navigation;

    public ImageAlignmentHelper(int image_width, JennyNavigation nav, LinearOpMode m){
        imagePid = new PIDController(MOVEMENT_KP_UNSCALED/image_width,MOVEMENT_KI,0);
        imagePid.setIMax(MOVEMENT_KI_MAX);
        mode = m;
        navigation = nav;
    }

    public double getKp(){
        return imagePid.getP();
    }

    public double getKi(){
        return imagePid.getI();
    }

    public double getKd(){
        return imagePid.getD();
    }

    public double getKiMax(){
        return imagePid.getI_Max();
    }

    public void setKp(double kP){
        imagePid.setP(kP);
    }

    public void setKi(double kI){
        imagePid.setI(kI);
    }

    public void setKd(double Kd){
        imagePid.setD(Kd);
    }

    public void setKiMax(double iMax){
        imagePid.setIMax(iMax);
    }


    public int findCryptoColumnClosestToDesired(ArrayList<Integer> columns, int desiredLocationOnScreen){
        if(columns.size() == 0){
            return -1;
        }
        int minDelta = Math.abs(desiredLocationOnScreen - columns.get(0));
        int minDeltaIndex = 0;
        for(int i = 1; i < columns.size(); i ++){
            if(Math.abs(desiredLocationOnScreen - columns.get(i)) < minDelta){
                minDelta = Math.abs(desiredLocationOnScreen - columns.get(i));
                minDeltaIndex = i;
            }
        }
        Log.d("Target Column","" + minDeltaIndex);
        return minDeltaIndex;
    }

    public boolean centerOnCryptoBoxClosestToCenter(int column, ArrayList<Integer> columns, int primaryDirection, int secondaryDirection){
        Log.d("Seen columns", Integer.toString(columns.size()));
        if(columns.size() == 0){
            navigation.correctedDriveOnHeadingIMU(primaryDirection, SLOW_SPEED_IN_PER_SEC, 0, mode);
            Log.d("Seek Status","Not found, moving in primary dir");
            return false;
        }
        int targetColumnIndex = findCryptoColumnClosestToDesired(columns, CRYPTO_COLUMN_TARGET_POSITION);
        if(targetColumnIndex != -1) {
            if (Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION) > DESIRED_PIXEL_ACCURACY) {
                Log.d("Delta Pixels","" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                //keep driving the hint
                Log.d("Column location", columns.get(column).toString());
                imagePid.setSp(0);
                double distToColumn = columns.get(column) - CRYPTO_COLUMN_TARGET_POSITION;
                double velocityCorrection = -imagePid.calculatePID(distToColumn);
                Log.d("Seek Status","Found, moving at " + velocityCorrection);
                double degreeToTravelPrimaryDir = navigation.getOrientation() + 90;
                navigation.correctedDriveOnHeadingIMU((velocityCorrection < 0) ? degreeToTravelPrimaryDir : degreeToTravelPrimaryDir + 180.0, Math.abs(velocityCorrection), mode);
            } else {
                navigation.brake();
                Log.d("Seek Status", "Obtained, delta Pixels:" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                return true;
            }
        }
        return false;
    }
    /*
    public boolean centerOnCryptoBoxClosestToCenter(int column, ArrayList<Integer> columns, int primaryDirection, int secondaryDirection){
        Log.d("Seen columns", Integer.toString(columns.size()));
        if(columns.size() == 0){
            navigation.correctedDriveOnHeadingIMU(primaryDirection, SLOW_SPEED_IN_PER_SEC, 0, mode);
            Log.d("Seek Status","Not found, moving in primary dir");
            return false;
        }
        int targetColumnIndex = findCryptoColumnClosestToDesired(columns, CRYPTO_COLUMN_TARGET_POSITION);
        if(targetColumnIndex != -1) {
            if (Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION) > DESIRED_PIXEL_ACCURACY) {
                Log.d("Delta Pixels","" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                //keep driving the hint
                Log.d("Column location", columns.get(column).toString());
                imagePid.setSp(0);
                double distToColumn = columns.get(column) - CRYPTO_COLUMN_TARGET_POSITION;
                double velocityCorrection = -imagePid.calculatePID(distToColumn);
                Log.d("Seek Status","Found, moving at " + velocityCorrection);
                navigation.correctedDriveOnHeadingIMU((velocityCorrection < 0) ? primaryDirection : secondaryDirection, Math.abs(velocityCorrection), mode);
            } else {
                navigation.brake();
                Log.d("Seek Status", "Obtained, delta Pixels:" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                return true;
            }
        }
        return false;
    }
     */
}
