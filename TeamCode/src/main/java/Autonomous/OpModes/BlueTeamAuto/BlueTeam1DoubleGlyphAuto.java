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
package Autonomous.OpModes.BlueTeamAuto;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.ArrayList;

import Actions.JennyO1BGlyphPicker;
import Actions.JennyO1CRAD;
import Actions.JewelJousterV2;
import Actions.NewArialDepositor;
import Autonomous.ImageAlignmentHelper;
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
import static Autonomous.REVColorDistanceSensorController.color.RED;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_1;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.RED_ALLIANCE_1;
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
@Autonomous(name="Blue Team 1 Double Glyph", group="Two Glyph Visual")  // @Autonomous(...) is the other common choice
//@Disabled
public class BlueTeam1DoubleGlyphAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JennyO1CRAD rad;
    NewArialDepositor glyphSystem;
    JennySensorTelemetry sensorTelemetry;
    JewelJousterV2 jewelJouster;
    VuforiaHelper vuforia;
    RelicRecoveryVuMark mark;
    JennyO1BGlyphPicker glyphPicker;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;
    ImageAlignmentHelper cryptoBoxAligner;
    //ImuHandler imuHandler;
    @Override
    public void runOpMode() {
        //imuHandler = new ImuHandler("imu", hardwareMap);
        int redCount = 0;
        int blueCount = 0;
        REVColorDistanceSensorController.color jewelColor;
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_1], WEST, "RobotConfig/JennyV2.json");
            glyphSystem = new NewArialDepositor(hardwareMap);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            jewelJouster = new JewelJousterV2("jewelJoust", "jewelJoustTurn", this, hardwareMap);
            vuforia = new VuforiaHelper();
            cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT, DESIRED_WIDTH, CLOSE_UP_MIN_PERCENT_COLUMN_CHECK, CLOSE_UP_MIN_COLUMN_WIDTH, CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE);
            cryptoBoxAligner = new ImageAlignmentHelper(DESIRED_WIDTH, navigation, this);
            rad = new JennyO1CRAD(hardwareMap);
            glyphPicker = new JennyO1BGlyphPicker(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        Bitmap curImage = null;
        ArrayList<Integer> columns;
        vuforia.loadCipherAssets();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        jewelJouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.READ);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        for(int i = 0; i < 5; i++){
            if(jewelJouster.getJewelColor() == BLUE) blueCount++;
            else if(jewelJouster.getJewelColor() == RED) redCount++;
        }
        jewelColor = (blueCount > redCount) ? BLUE:RED;
        if(jewelColor == RED){
            jewelJouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.HIT_LEFT);
            telemetry.addData("Jewel Color","RED");
        }
        else {
            //navigation.driveDistance(2, NORTH, ADJUSTING_SPEED_IN_PER_SEC, this);
            jewelJouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.HIT_RIGHT);
            telemetry.addData("Jewel Color","BLUE");
            //sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        }
        telemetry.update();
        sleep(MED_SLEEP_DELAY_MILLIS);
        jewelJouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        telemetry.update();
        navigation.turnToHeading(250, this);
        navigation.brake();
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        long startTime = System.currentTimeMillis();
        mark = vuforia.getMark();

        while(mark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive() && System.currentTimeMillis() - startTime < 1000){
            mark = vuforia.getMark();
        }

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
        if(mark == RelicRecoveryVuMark.UNKNOWN) mark = RelicRecoveryVuMark.CENTER;
        telemetry.update();
        navigation.turnToHeading(WEST, this);
        navigation.driveDistance(30,WEST,HIGH_SPEED_IN_PER_SEC,this);
        navigation.turnToHeading(WEST + 45,this);
        if(opModeIsActive()) glyphPicker.grab();
        if(opModeIsActive()) glyphSystem.startBelt();
        //navigation.driveDistance(3,NORTH + 45,SLOW_SPEED_IN_PER_SEC,this);
        //sleep(DEFAULT_DELAY_MILLIS);
        //navigation.brake();
        getSecondGlyph();
        navigation.turnToHeading(NORTH,this);
        if(opModeIsActive()) glyphPicker.spit();
        navigation.driveDistance(12,SOUTH,HIGH_SPEED_IN_PER_SEC,this);
        navigation.driveDistance(5,SOUTH,MED_SPEED_IN_PER_SEC,this);
        if(opModeIsActive()) glyphSystem.stopBelt();
        if(opModeIsActive()) glyphPicker.pause();
        switch (mark) {
            case CENTER:
                navigation.driveDistance(16.5, EAST, SLOW_SPEED_IN_PER_SEC, this);
                break;
            case LEFT:
                navigation.driveDistance(20, EAST, SLOW_SPEED_IN_PER_SEC, this);
                break;
            case RIGHT:
                navigation.driveDistance(10.5, EAST, SLOW_SPEED_IN_PER_SEC, this);
                break;
        }
        navigation.brake();
        //navigation.driveDistance(28, SOUTH, SLOW_SPEED_IN_PER_SEC, this);
        //navigation.turnToHeading(NORTH, this);
        navigation.driveDistance(15, SOUTH, MED_SPEED_IN_PER_SEC, this);
        navigation.brake();
        navigation.driveDistance(4.5, NORTH, SLOW_SPEED_IN_PER_SEC, this);
        navigation.turnToHeading(NORTH,this);
        sleep(DEFAULT_DELAY_MILLIS);
        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        columns = cryptoBoxFinder.findColumns(curImage, false);
        while (!cryptoBoxAligner.centerOnCryptoBoxClosestToCenter(0, columns, WEST, EAST) && opModeIsActive()) {
            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            columns = cryptoBoxFinder.findColumns(curImage, false);
        }
        navigation.brake();
        curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        columns = cryptoBoxFinder.findColumns(curImage, false);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        while (!cryptoBoxAligner.centerOnCryptoBoxClosestToCenter(0, columns, WEST, EAST) && opModeIsActive()) {
            curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
            columns = cryptoBoxFinder.findColumns(curImage, false);
        }
        navigation.brake();
        if(opModeIsActive())glyphSystem.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW2);
        if(opModeIsActive())sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        if(opModeIsActive())glyphSystem.startBelt();
        if(opModeIsActive())sleep(1500);
        if(opModeIsActive())navigation.driveDistance(6, NORTH, SLOW_SPEED_IN_PER_SEC, this);
        if(opModeIsActive())glyphSystem.stopBelt();
        while(!glyphSystem.isPressed() && opModeIsActive()){
            glyphSystem.retract();
        }
        glyphSystem.stopLift();
        while(opModeIsActive());
        navigation.stopNavigation();
        glyphSystem.kill();
        vuforia.kill();
//        glyphSystem.stopNavigation();
    }

    public boolean getSecondGlyph(){
        double turnMagnitude = 0;
        double driveMagnitude = 0;
        long startTime = System.currentTimeMillis();
        attemptGrab(NORTH);
        sleep(200);
        navigation.brake();

        return true;
    }

    public void attemptGrab(int dir){
        navigation.driveDistanceNonCorrected(11.5,dir,HIGH_SPEED_IN_PER_SEC,this);
//        navigation.relativeDriveOnHeadingWithTurning(dir,MED_SPEED_IN_PER_SEC,0);
        //sleep(delayGoingIn);
  //navigation.turnToHeading(navigation.getOrientation() - 20, this);
//        navigation.relativeDriveOnHeadingWithTurning((dir + 180)%360,MED_SPEED_IN_PER_SEC,0);
        navigation.driveDistanceNonCorrected(11.5,(dir + 180)%360,MED_SPEED_IN_PER_SEC,this);
//        sleep(goingOut);
        navigation.brake();
    }
}
