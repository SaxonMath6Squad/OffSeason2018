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

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.ArrayList;

import Actions.ArialDepositor;
import Actions.JewelJouster;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.REVColorDistanceSensorController;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.REVColorDistanceSensorController.color.RED;
import static Autonomous.REVColorDistanceSensorController.color.UNKNOWN;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.ADJUSTING_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.MED_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SLOW_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.SOUTH;
import static DriveEngine.JennyNavigation.WEST;

/*
    An opmode to test knocking off the correct jewel
 */
@Autonomous(name="New Center On Column", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class CenterOnCryptobox extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JennySensorTelemetry sensorTelemetry;
    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        Bitmap curImage = null;
        ArrayList<Integer> coloredColumns;
        long startTime = System.currentTimeMillis();
        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
        while (coloredColumns.size() == 0 && opModeIsActive()) {
            navigation.correctedDriveOnHeadingIMU(EAST, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
        }
        navigation.brake();

        while (!centerOnCryptoBox(0, coloredColumns, EAST, WEST) && opModeIsActive()) {
            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
        }
        navigation.brake();

        while(opModeIsActive());
        navigation.stopNavigation();
        sensorTelemetry.stopSensorTelemetry();
    }

    public boolean centerOnCryptoBox(int column, ArrayList<Integer> columns, int primaryDirection, int secondaryDirection){
        Log.d("Seen columns", Integer.toString(columns.size()));
        if(columns.size() == 0){
            navigation.correctedDriveOnHeadingIMU(primaryDirection, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
            return false;
        }
        else if(columns.get(column) -  DESIRED_WIDTH/2 < -DESIRED_WIDTH/8){
            //keep driving the hint
            Log.d("Column location", columns.get(column).toString());
            navigation.correctedDriveOnHeadingIMU(primaryDirection,ADJUSTING_SPEED_IN_PER_SEC,this);
        }
        else if(columns.get(column) -  DESIRED_WIDTH/2 > DESIRED_WIDTH/8){
            //keep driving the hint
            Log.d("Column location", columns.get(column).toString());
            navigation.correctedDriveOnHeadingIMU(secondaryDirection,ADJUSTING_SPEED_IN_PER_SEC,this);
        }
        else {
            Log.d("Column location", columns.get(column).toString());
            navigation.brake();
            return true;
        }
        return false;
    }
}
