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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.ArrayList;

import Actions.ArialDepositor;
import Actions.JewelJouster;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.ADJUSTING_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SOUTH;

/*
    An opmode to test knocking off the correct jewel
 */
@Autonomous(name="SinTest", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class RobotSinWave extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    ArialDepositor glyphSystem;
    JennySensorTelemetry sensorTelemetry;
    JewelJouster jewelJouster;
    VuforiaHelper vuforia;
    RelicRecoveryVuMark mark;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    //ImuHandler imuHandler;
    @Override
    public void runOpMode() {
        //imuHandler = new ImuHandler("imu", hardwareMap);
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 11, "RobotConfig/JennyV2.json");
            glyphSystem = new ArialDepositor(hardwareMap);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            jewelJouster = new JewelJouster("jewelJouster", hardwareMap);
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT, DESIRED_WIDTH, CLOSE_UP_MIN_PERCENT_COLUMN_CHECK, CLOSE_UP_MIN_COLUMN_WIDTH);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        vuforia.loadCipherAssets();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested());
        if(isStopRequested()){
            navigation.stopNavigation();
            return;
        }
        runtime.reset();
        long startTime = System.currentTimeMillis();
        while (opModeIsActive()){
            //lets create a panning motion....
            navigation.turnToHeading(215 - 20*Math.sin((System.currentTimeMillis() - startTime)/1000.0),this);
            mark = vuforia.getMark();
            switch (mark){
                case LEFT:
                    telemetry.addData("Mark", "LEFT");
                    break;
                case CENTER:
                    telemetry.addData("Mark", "CENTER");
                    break;
                case RIGHT:
                    telemetry.addData("Mark", "RIGHT");
                    break;
                case UNKNOWN:
                    telemetry.addData("Mark", "UNKNOWN");
                    break;
                default:
                    telemetry.addData("Mark", "HUH");
                    break;
            }
            telemetry.update();
        }

        navigation.stopNavigation();
//        glyphSystem.stopNavigation();
    }

    public boolean centerOnCryptoBox(int column, ArrayList<Integer> centers, int dirHint){
        if(cryptoBoxFinder.imageWidth/2 < centers.get(column).intValue()){
            if(cryptoBoxFinder.imageWidth/2  - centers.get(column).intValue() < centers.get(column).intValue()/10){
                navigation.newDriveOnHeadingIMU(SOUTH, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
            } else {
                navigation.brake();
                return true;
            }
        } else if(cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()){
            if(centers.get(column).intValue() - cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()/10){
                navigation.newDriveOnHeadingIMU(NORTH, ADJUSTING_SPEED_IN_PER_SEC, 0, this);
            } else {
                navigation.brake();
                return true;
            }
        } else {
            navigation.brake();
            return true;
        }
        return false;
    }
}
