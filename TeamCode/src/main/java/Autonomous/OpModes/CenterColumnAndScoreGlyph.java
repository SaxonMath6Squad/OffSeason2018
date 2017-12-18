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
import DriveEngine.JennyNavigation;
import Systems.JennyV2PickAndExtend;
import Autonomous.VuforiaHelper;


import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SOUTH;
import static DriveEngine.JennyNavigation.WEST;

@Autonomous(name="Score glyph with camera test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class CenterColumnAndScoreGlyph extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JennyV2PickAndExtend glyphSystem;
    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor columnFinder;
    //ImuHandler imuHandler;
    @Override
    public void runOpMode() {
        //imuHandler = new ImuHandler("imu", hardwareMap);
        try {
            navigation = new JennyNavigation(hardwareMap,"RobotConfig/JennyV2.json");
            glyphSystem = new JennyV2PickAndExtend(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        vuforia = new VuforiaHelper();
        columnFinder = new CryptoBoxColumnImageProcessor(171,244,.1,1);
        Bitmap bmp;
        ArrayList<Integer> columnLocations = new ArrayList<Integer>();
        boolean centered = false;
        boolean finished = false;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while(opModeIsActive() && !finished){
            bmp = vuforia.getImage(171,244);
            if(bmp != null){
                columnLocations = columnFinder.findColumns(bmp, false);
                if(columnLocations != null){
                    for(int i = 0; i < columnLocations.size(); i ++){
                        telemetry.addData("Column " + i, " " + columnLocations.get(i).intValue());
                    }
                    telemetry.update();
                    if(!centered && columnLocations.size() > 2){
                        int columnCenter = (int)((columnLocations.get(0) + columnLocations.get(1)) / 2 + .5);
                        centerSelfOnColumn(columnCenter, bmp.getWidth() / 2);
                        centered = columnFinder.isCentered(columnCenter, bmp.getWidth() / 2);
                    }
                    if(centered){
                        navigation.driveDistance(3*12, NORTH, 15, this);
                        sleep(75);
                        glyphSystem.reverseGlyphBelt();
                        glyphSystem.spit();
                        sleep(250);
                        glyphSystem.pauseBelt();
                        glyphSystem.pauseGrabber();
                        navigation.driveDistance(3, SOUTH, 15, this);
                        sleep(75);
                        navigation.driveDistance(6, EAST, 15, this);
                        sleep(75);
                        finished = true;
                    }
                }
            } else {
                Log.d("BMP", "NULL!");
            }
        }
        
        navigation.brake();
        navigation.stop();
        glyphSystem.stop();
    }
    
    private void centerSelfOnColumn(int desiredColumnCenter, int imageCenter){
        if(imageCenter > desiredColumnCenter){
            if(imageCenter - desiredColumnCenter > 20){
                navigation.driveOnHeadingIMU(WEST, 10, this);
            } else {
                navigation.brake();
            }
        } else if(imageCenter < desiredColumnCenter){
            if(desiredColumnCenter - imageCenter > 20){
                navigation.driveOnHeadingIMU(EAST, 10, this);
            } else {
                navigation.brake();
            }
        } else {
            navigation.brake();
        }
    }
}
