/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package Autonomous.OpModes;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import MotorControllers.PIDController;
import SensorHandlers.JennySensorTelemetry;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.ADJUSTING_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.SLOW_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.WEST;

/*
    An opmode to test knocking off the correct jewel
 */
@Autonomous(name="New Center On Column PID", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class CenterOnCryptoboxPID extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JennySensorTelemetry sensorTelemetry;
    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    PIDController cameraPIDController;

    int CRYPTO_COLUMN_TARGET_POSITION = 67;

    @Override
    public void runOpMode() {
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT, DESIRED_WIDTH, CLOSE_UP_MIN_PERCENT_COLUMN_CHECK, CLOSE_UP_MIN_COLUMN_WIDTH, CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        cameraPIDController = new PIDController(5.0/DESIRED_WIDTH,0,0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Bitmap curImage = null;
        ArrayList<Integer> coloredColumns;
        long startTime = System.currentTimeMillis();
        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
        while(!centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive()){
            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
        }
        sleep(2000);
        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
        int targetColumnIndex = findCryptoColumnClosestToDesired(coloredColumns, CRYPTO_COLUMN_TARGET_POSITION);
        telemetry.addData("Seek Status", "Obtained, delta Pixels:" + Math.abs(coloredColumns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
        telemetry.update();
//        while (coloredColumns.size() == 0 && opModeIsActive()) {
//            navigation.correctedDriveOnHeadingIMU(EAST, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
//            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
//            coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
//        }
//        navigation.brake();
//
//        while (!centerOnCryptoBox(0, coloredColumns, EAST, WEST) && opModeIsActive()) {
//            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
//            coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
//        }
        navigation.brake();

        while(opModeIsActive());
        navigation.stopNavigation();
        sensorTelemetry.stopSensorTelemetry();
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
            navigation.correctedDriveOnHeadingIMU(primaryDirection, SLOW_SPEED_IN_PER_SEC, 0, this);
            Log.d("Seek Status","Not found, moving in primary dir");
            return false;
        }
        int targetColumnIndex = findCryptoColumnClosestToDesired(columns, CRYPTO_COLUMN_TARGET_POSITION);
        if(targetColumnIndex != -1) {
            if (Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION) > DESIRED_WIDTH/20.0) {
                Log.d("Delta Pixels","" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                //keep driving the hint
                Log.d("Column location", columns.get(column).toString());
                cameraPIDController.setSp(0);
                double distToColumn = columns.get(column) - CRYPTO_COLUMN_TARGET_POSITION;
                double velocityCorrection = -cameraPIDController.calculatePID(distToColumn);
                Log.d("Seek Status","Found, moving at " + velocityCorrection);
                navigation.correctedDriveOnHeadingIMU((velocityCorrection < 0) ? primaryDirection : secondaryDirection, Math.abs(velocityCorrection), this);
            } else {
                navigation.brake();
                Log.d("Seek Status", "Obtained, delta Pixels:" + Math.abs(columns.get(targetColumnIndex) - CRYPTO_COLUMN_TARGET_POSITION));
                return true;
            }
        }
        return false;
    }
}
