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
package Testers;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import Actions.ArialDepositor;
import Actions.JennyO1BGlyphPicker;
import Actions.JewelJouster;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.REVColorDistanceSensorController.*;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Actions.JennyFlagController;
import Actions.JennyO1BRAD;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.LONG_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.MED_SLEEP_DELAY_MILLIS;
import static SensorHandlers.JennySensorTelemetry.COLOR_DISTANCE_SENSOR;

import static SensorHandlers.JennySensorTelemetry.RAD_LIMIT;

/*
    An opmode to test if all our drive wheels are working correctly
 */
@TeleOp(name="Auto Wiring Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class AutoWiringTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JennyO1BRAD rad;
    ArialDepositor glyphPlacement;
    JennySensorTelemetry sensorTelemetry;
    JennyFlagController flagController;
    JewelJouster jewelJouster;
    JennyO1BGlyphPicker glyphPicker;
    CryptoBoxColumnImageProcessor cryptoboxFinder;
    VuforiaHelper vuforia;

    @Override
    public void runOpMode() {
        boolean[] driveMotorOk = {false, false, false, false};
        int[] driveMotorCount = {0, 0, 0, 0};
        boolean miscMotorOk = false;
        int miscMotorCount = 0;
        boolean[] sensorOk = {false, false, false};

        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
            rad = new JennyO1BRAD(hardwareMap);
            glyphPlacement = new ArialDepositor(hardwareMap);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            flagController = new JennyFlagController(hardwareMap);
            jewelJouster = new JewelJouster("jewelJouster", hardwareMap);
            glyphPicker = new JennyO1BGlyphPicker(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , e.toString());
            throw new RuntimeException("System Creation Error! " + e.toString());

        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // DRIVE
        double[] motorPositions = navigation.getMotorPositionsInches();
        navigation.applyMotorPowers(new double[]{0.3, 0.3, 0.3, 0.3});
        sleep(MED_SLEEP_DELAY_MILLIS);
        navigation.brake();
        double[] newMotorPositions = navigation.getMotorPositionsInches();
        for(int i = 0; i < navigation.driveMotors.length; i++) {
            if (newMotorPositions[i] > motorPositions[i]) driveMotorCount[i]++;
        }
        motorPositions = navigation.getMotorPositionsInches();
        navigation.applyMotorPowers(new double[]{-0.3, -0.3, -0.3, -0.3});
        sleep(MED_SLEEP_DELAY_MILLIS);
        navigation.brake();
        newMotorPositions = navigation.getMotorPositionsInches();
        for(int i = 0; i < navigation.driveMotors.length; i++) {
            if (motorPositions[i] > newMotorPositions[i]) driveMotorCount[i]++;
            if (driveMotorCount[i] == 2) driveMotorOk[i] = true;
            if (driveMotorOk[i]){
                telemetry.addData("Drive Motor " + Integer.toString(i), "OK!");
                telemetry.update();
                sleep(MED_SLEEP_DELAY_MILLIS);
            }
        }
        //DRIVE

        //TODO: USE EXTEND TO POSITION INSTEAD, BRINGING IT TO THE TOP
        //EXTEND
        boolean liftSwitch = sensorTelemetry.isPressed(RAD_LIMIT);
        double liftPosition = glyphPlacement.getLiftMotorPosition();
        glyphPlacement.setLiftPower(0.3);
        sleep(LONG_SLEEP_DELAY_MILLIS);
        glyphPlacement.stopLift();
        if(liftSwitch && !sensorTelemetry.isPressed(RAD_LIMIT)) liftSwitch = true;
        else liftSwitch = false;
        double newLiftPosition = glyphPlacement.getLiftMotorPosition();
        if(newLiftPosition > liftPosition) miscMotorCount++;
        liftPosition = glyphPlacement.getLiftMotorPosition();
        glyphPlacement.setLiftPower(-0.3);
        sleep(MED_SLEEP_DELAY_MILLIS);
        glyphPlacement.stopLift();
        newLiftPosition = glyphPlacement.getLiftMotorPosition();
        if(liftPosition > newLiftPosition) miscMotorCount++;
        if(miscMotorCount == 2) miscMotorOk = true;
        if(miscMotorOk) {
            telemetry.addData("Misc Motor (Lift motor)", "OK!");
            telemetry.update();
            sleep(MED_SLEEP_DELAY_MILLIS);
        } else {
            telemetry.addData("Misc Motor (Lift motor)", "BAD!");
            telemetry.update();
        }
        //EXTEND

        //COLOR SENSORS
        jewelJouster.setPosition(JewelJouster.EXTENDION_MODE.HIT);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        color[] detectedColors = new color[3];
        //detectedColors[0] = sensorTelemetry.getColor(COLOR_DISTANCE_SENSOR);
        detectedColors[1] = jewelJouster.getJewelColor();
        detectedColors[2] = glyphPlacement.getColor();
        for(int i = 0; i < detectedColors.length; i++){
            if(detectedColors[i] == color.UNKNOWN) sensorOk[i] = true;
            if(sensorOk[i]){
                telemetry.addData("Color Sensor " + Integer.toString(i), "OK!");
                telemetry.update();
                sleep(MED_SLEEP_DELAY_MILLIS);
            }
        }
        jewelJouster.setPosition(JewelJouster.EXTENDION_MODE.STORE);
        //COLOR SENSORS

        //RAD
        rad.releaseRelic();
        sleep(MED_SLEEP_DELAY_MILLIS);
        rad.stopRelic();
        boolean radOk = sensorTelemetry.isPressed(RAD_LIMIT);
        rad.extendRAD();
        sleep(MED_SLEEP_DELAY_MILLIS);
        rad.pauseRADExtender();
        if(radOk && !sensorTelemetry.isPressed(RAD_LIMIT)) radOk = true;
        else radOk = false;
        rad.retractRAD();
        sleep(MED_SLEEP_DELAY_MILLIS);
        rad.pauseRADExtender();
        rad.grabRelic();
        sleep(MED_SLEEP_DELAY_MILLIS + DEFAULT_SLEEP_DELAY_MILLIS);
        rad.stopRelic();
        if(radOk){
            telemetry.addData("RAD & limit", "OK!");
        } else {
            telemetry.addData("RAD", "BAD!");
        }
        telemetry.update();
        //RAD

        //GLYPH PICKER
        glyphPicker.grab();
        sleep(MED_SLEEP_DELAY_MILLIS);
        glyphPicker.spit();
        sleep(MED_SLEEP_DELAY_MILLIS);
        glyphPicker.pause();
        //GLYPH PICKER

        //FLAG
        flagController.startFlag();
        sleep(LONG_SLEEP_DELAY_MILLIS);
        flagController.stopFlag();
        //FLAG

        //JEWEL JOUST
        jewelJouster.setPosition(JewelJouster.EXTENDION_MODE.HIT);
        sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        jewelJouster.setPosition(JewelJouster.EXTENDION_MODE.STORE);
        //JEWEL JOUST

        //SWITCHES DOUBLE CHECK
        telemetry.addData("Press", "Extend Limit");
        telemetry.update();
        sleep(MED_SLEEP_DELAY_MILLIS);
        if(sensorTelemetry.isPressed(RAD_LIMIT)){
            telemetry.addData("Release", "Extend Limit");
        } else {
            telemetry.addData("Error", "Extend Limit not pressed!");
        }
        telemetry.update();
        sleep(MED_SLEEP_DELAY_MILLIS);
        if(!sensorTelemetry.isPressed(RAD_LIMIT)){
            telemetry.addData("Extend Limit", "OK!");
        } else {
            telemetry.addData("Error", "Extend Limit not released!");
        }
        telemetry.update();
        sleep(MED_SLEEP_DELAY_MILLIS);
        telemetry.addData("Press", "RAD Limit");
        telemetry.update();
        sleep(MED_SLEEP_DELAY_MILLIS);
        if(sensorTelemetry.isPressed(RAD_LIMIT)){
            telemetry.addData("Release", "RAD Limit");
        } else {
            telemetry.addData("Error", "RAD Limit not pressed!");
        }
        telemetry.update();
        sleep(MED_SLEEP_DELAY_MILLIS);
        if(!sensorTelemetry.isPressed(RAD_LIMIT)){
            telemetry.addData("RAD Limit", "OK!");
        } else {
            telemetry.addData("Error", "RAD Limit not released1");
        }
        telemetry.update();
        //SWITCHES DOUBLE CHECK

        //CAMERA
        Bitmap bmp = null;
        ArrayList<Integer> columns;
        ArrayList<Integer> centers;
        bmp = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
        if(bmp == null){
            telemetry.addData("Error", "Unable to take image");
            telemetry.update();
            sleep(MED_SLEEP_DELAY_MILLIS);
        } else {
            columns = cryptoboxFinder.findColumns(bmp, true);
            vuforia.saveBMP(bmp);
            centers = cryptoboxFinder.findColumnCenters(bmp, true);
            vuforia.saveBMP(bmp);
            telemetry.addData("Images created", "Please check");
            telemetry.update();
            sleep(MED_SLEEP_DELAY_MILLIS);
        }
        //CAMERA

        navigation.stopNavigation();
        rad.stop();
        glyphPicker.stop();
        glyphPlacement.stop();
        jewelJouster.stop();
        sensorTelemetry.stopSensorTelemetry();
        flagController.stopFlag();
    }
}
