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

import Actions.JennyO1BGlyphPicker;
import Autonomous.GlyphStackingEngine;
import Actions.JennyO1CRAD;
import Actions.JewelJousterV2;
import Actions.NewArialDepositor;
import Autonomous.ImageAlignmentHelper;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.REVColorDistanceSensorController;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Autonomous.Location;

import static Autonomous.GlyphStackingEngine.Glyph.GRAY;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.REVColorDistanceSensorController.color.BLUE;
import static Autonomous.REVColorDistanceSensorController.color.GREY;
import static Autonomous.REVColorDistanceSensorController.color.NOT_IN_RANGE;
import static Autonomous.REVColorDistanceSensorController.color.RED;
import static Autonomous.REVColorDistanceSensorController.color.UNKNOWN;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_1;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.DEFAULT_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.HIGH_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.MED_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.MED_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SLOW_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.SOUTH;
import static DriveEngine.JennyNavigation.WEST;

/*
    An opmode to test knocking off the correct jewel
 */
@Autonomous(name="Two glyph test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TwoGlyphAutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    NewArialDepositor glyphSystem;
    JennyO1BGlyphPicker glyphPicker;
    GlyphStackingEngine cipherTracker;
    JennySensorTelemetry sensorTelemetry;
    JewelJousterV2 jewelJouster;
    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    ImageAlignmentHelper cryptoBoxAligner;
    JennyO1CRAD rad;

    @Override
    public void runOpMode() {
        Bitmap curImage = null;
        ArrayList<Integer> columns = new ArrayList<>();


        REVColorDistanceSensorController.color currentColor = UNKNOWN;
        long startTime = System.currentTimeMillis();

        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], WEST, "RobotConfig/JennyV2.json");
            glyphSystem = new NewArialDepositor(hardwareMap);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            jewelJouster = new JewelJousterV2("jewelJoust", "jewelJoustTurn", this, hardwareMap);
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT, DESIRED_WIDTH, CLOSE_UP_MIN_PERCENT_COLUMN_CHECK, CLOSE_UP_MIN_COLUMN_WIDTH, CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE);
            cryptoBoxAligner = new ImageAlignmentHelper(DESIRED_WIDTH, navigation, this);
            rad = new JennyO1CRAD(hardwareMap);
            glyphPicker = new JennyO1BGlyphPicker(hardwareMap);
            cipherTracker = new GlyphStackingEngine();
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        cipherTracker.placeGlyph(GREY, new Location(1, 0));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // get second glyph
        glyphPicker.grab();
        glyphSystem.startBelt();
        navigation.driveDistance(30, WEST, HIGH_SPEED_IN_PER_SEC, this);
        getSecondGlyph();

        currentColor = glyphSystem.getColor(NewArialDepositor.REAR_GLYPH_SENSOR);

        // drive back and center
        navigation.driveDistance(30, EAST, MED_SPEED_IN_PER_SEC, this);
        glyphPicker.pause();
        glyphSystem.stopBelt();

        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        columns = cryptoBoxFinder.findColumns(curImage, false);
        while (!cryptoBoxAligner.centerOnCryptoBoxClosestToCenter(0, columns, NORTH, SOUTH) && opModeIsActive()){
            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            columns = cryptoBoxFinder.findColumns(curImage, false);
        }
        navigation.brake();

        // place second glyph
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        glyphSystem.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW2);
        cipherTracker.placeGlyph(currentColor, new Location(1, 1));
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        glyphSystem.startBelt();
        sleep(2000);
        navigation.driveDistance(5, WEST, SLOW_SPEED_IN_PER_SEC, this);
        glyphSystem.stopBelt();
        glyphSystem.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.GROUND);
        sleep(DEFAULT_DELAY_MILLIS);

        // get third glyph and go back
//        glyphPicker.grab();
//        glyphSystem.startBelt();
//        glyphColor = glyphSystem.getColor(NewArialDepositor.REAR_GLYPH_SENSOR);
//        navigation.driveDistance(30, WEST, HIGH_SPEED_IN_PER_SEC, this);
//        while ((glyphColor == UNKNOWN || glyphColor == NOT_IN_RANGE) && opModeIsActive()){
//            turnMagnitude = Math.sin((System.currentTimeMillis() - startTime / 1000.0)) * .25;
//            navigation.driveOnHeadingWithTurning(WEST, MED_SLEEP_DELAY_MILLIS, turnMagnitude);
//            glyphColor = glyphSystem.getColor(NewArialDepositor.REAR_GLYPH_SENSOR);
//        }
//        currentColor = glyphSystem.getColor(NewArialDepositor.REAR_GLYPH_SENSOR);
//        navigation.brake();
//        navigation.driveDistance(30, EAST, HIGH_SPEED_IN_PER_SEC, this);
//        glyphSystem.stopBelt();
//        glyphPicker.pause();
    }

    public boolean getSecondGlyph(){
        REVColorDistanceSensorController.color glyphColor = UNKNOWN;
        double turnMagnitude = 0;
        double driveMagnitude = 0;
        long startTime = System.currentTimeMillis();
        while ((glyphColor == UNKNOWN || glyphColor == NOT_IN_RANGE) && opModeIsActive()){
            turnMagnitude = Math.sin((System.currentTimeMillis() - startTime / 1000.0)) * .25;
            driveMagnitude = Math.sin((System.currentTimeMillis() - startTime / 10000.0)) * 10;
            if(driveMagnitude < 0)
                navigation.relativeDriveOnHeadingWithTurning(EAST, driveMagnitude, 0);
            else
                navigation.relativeDriveOnHeadingWithTurning(WEST, driveMagnitude, 0);
            glyphColor = glyphSystem.getColor(NewArialDepositor.REAR_GLYPH_SENSOR);
        }
        return true;
    }
}
