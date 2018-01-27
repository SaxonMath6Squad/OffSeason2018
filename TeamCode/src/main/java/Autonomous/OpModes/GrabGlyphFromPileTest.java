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
import Actions.HardwareWrappers.NewArialDepositor;
import Actions.JennyO1BGlyphPicker;
import Actions.JewelJouster;
import Actions.JewelJousterV2;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.REVColorDistanceSensorController;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.REVColorDistanceSensorController.color.BLUE;
import static Autonomous.REVColorDistanceSensorController.color.NOT_IN_RANGE;
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
@Autonomous(name="Get Glyph From Pile Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class GrabGlyphFromPileTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    NewArialDepositor glyphSystem;
    JennySensorTelemetry sensorTelemetry;
    JewelJousterV2 jewelJouster;
    JennyO1BGlyphPicker glyphPicker;
    double turnRPS = 0;
    //ImuHandler imuHandler;
    @Override
    public void runOpMode() {
        //imuHandler = new ImuHandler("imu", hardwareMap);
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 9, "RobotConfig/JennyV2.json");
            glyphSystem = new NewArialDepositor(hardwareMap);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            jewelJouster = new JewelJousterV2("jewelJoust", "jewelJoustTurn", this, hardwareMap);
            glyphPicker = new JennyO1BGlyphPicker(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        REVColorDistanceSensorController.color rearColor = UNKNOWN;
        REVColorDistanceSensorController.color frontColor = UNKNOWN;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        navigation.driveDistance(24, EAST, MED_SPEED_IN_PER_SEC, this);
        glyphPicker.grab();
        glyphSystem.startBelt();
        rearColor = glyphSystem.getColor(NewArialDepositor.REAR_GLYPH_SENSOR);
        frontColor = glyphSystem.getColor(NewArialDepositor.FRONT_GLYPH_SENSOR);
        while ((rearColor == UNKNOWN || rearColor == NOT_IN_RANGE) && (frontColor == UNKNOWN || frontColor == NOT_IN_RANGE)) {
            for (int i = 0; i < 360; i++) {
                turnRPS = Math.sin(Math.toRadians(i)) * 0.05;
                navigation.driveOnHeadingWithTurning(EAST, ADJUSTING_SPEED_IN_PER_SEC, turnRPS);
            }
            rearColor = glyphSystem.getColor(NewArialDepositor.REAR_GLYPH_SENSOR);
            frontColor = glyphSystem.getColor(NewArialDepositor.FRONT_GLYPH_SENSOR);
        }
        navigation.driveDistance(24, WEST, MED_SPEED_IN_PER_SEC, this);
        while(opModeIsActive());
        navigation.stopNavigation();
        glyphSystem.kill();
//        glyphSystem.stopNavigation();
    }
}
