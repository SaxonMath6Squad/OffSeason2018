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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Systems.JennyV2PickAndExtend;
import Autonomous.VuforiaHelper;


import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SOUTH;
import static DriveEngine.JennyNavigation.WEST;

@Autonomous(name="Camera autonomous reversed", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class CenterColumnAndScoreGlyphReversed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JennyV2PickAndExtend glyphSystem;
    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    JennySensorTelemetry sensorTelemetry;
    //ImuHandler imuHandler;
    @Override
    public void runOpMode() {
        //imuHandler = new ImuHandler("imu", hardwareMap);
        try {
            navigation = new JennyNavigation(hardwareMap,"RobotConfig/JennyV2.json");
            glyphSystem = new JennyV2PickAndExtend(hardwareMap);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        vuforia = new VuforiaHelper();
        cryptoBoxFinder = new CryptoBoxColumnImageProcessor(171,244,.1,1);
        Bitmap bmp = null;
        ArrayList<Integer> columnLocations = new ArrayList<Integer>();
        boolean centered = false;
        boolean finished = false;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        navigation.driveDistance(2.25*12, EAST, 15, this);
        bmp = vuforia.getImage(171, 244);
        if(bmp == null){
            throw new RuntimeException("Image capture failed!");
        }
        columnLocations = cryptoBoxFinder.findColumnCenters(bmp, false);
        long startTime = System.currentTimeMillis();
        while(opModeIsActive() && !centerOnCryptoBox(0, columnLocations, WEST));
        Log.d("Time taken to center", Long.toString(System.currentTimeMillis() - startTime));
        double distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
        while (opModeIsActive() && (distToWall > 28 || distToWall == 0)){
            Log.d("Distance to wall", Double.toString(sensorTelemetry.getDistance(DistanceUnit.CM)));
            navigation.driveOnHeadingIMU(SOUTH, 5, this);
            distToWall = sensorTelemetry.getDistance(DistanceUnit.CM);
        }
        navigation.brake();
//        glyphSystem.lift();
        sleep(2500);
//        glyphSystem.pauseLift();
        glyphSystem.startGlyphBelt();
        sleep(3500);
        glyphSystem.pauseBelt();
//        glyphSystem.drop();
//        while (!sensorTelemetry.getState(sensorTelemetry.EXTEND_LIMIT));
//        glyphSystem.pauseLift();
        navigation.driveDistance(3, NORTH, 10, this);
        navigation.brake();
        navigation.stopNavigation();
        glyphSystem.stop();
    }

    public boolean centerOnCryptoBox(int column, ArrayList<Integer> centers, int dirHint){
        if(cryptoBoxFinder.imageWidth/2 < centers.get(column).intValue()){
            if(cryptoBoxFinder.imageWidth/2  - centers.get(column).intValue() < centers.get(column).intValue()/10){
                navigation.driveOnHeadingIMU(WEST, 5, this);
            } else {
                navigation.brake();
                return true;
            }
        } else if(cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()){
            if(centers.get(column).intValue() - cryptoBoxFinder.imageWidth/2  > centers.get(column).intValue()/10){
                navigation.driveOnHeadingIMU(EAST, 5, this);
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
