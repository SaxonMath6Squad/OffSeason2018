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
package UserControlled;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import Actions.ArialDepositor;
import Actions.JennyFlagController;
import Actions.JennyO1BGlyphPicker;
import Actions.JewelJouster;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import MotorControllers.PIDController;
import SensorHandlers.JennySensorTelemetry;
import Actions.JennyO1BRAD;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.CRYPTOBOX_COLORED_COLUMN_WIDTHS_INCHES;
import static Autonomous.RelicRecoveryField.CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.HIGH_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SLOW_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.WEST;
import static SensorHandlers.JennySensorTelemetry.RAD_LIMIT;

/*
    An opmode for the User Controlled portion of the game
    This code was used for the first regional qualifier at Asheville NC
    Limited autonomous, no ciphers on red team, glyphs close but not always scored.
    We won 1st place
 */
@TeleOp(name="Jenny O1B User Controlled", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class JennyO1B extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JoystickHandler leftJoystick, rightJoystick;
    ArialDepositor glyphLift;
    JennyO1BGlyphPicker glyphPicker;
    JennyO1BRAD RAD;
    JewelJouster jouster;
    JennySensorTelemetry sensorTelemetry;
    JennyFlagController flagController;
    boolean autoLiftPositionMode = false;
    boolean paralaxedControl = false;
    boolean flagOn = false;
    ArialDepositor.GLYPH_PLACEMENT_LEVEL[] liftPosition = new ArialDepositor.GLYPH_PLACEMENT_LEVEL[]{ArialDepositor.GLYPH_PLACEMENT_LEVEL.GROUND, ArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW1_AND_2, ArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW3_AND_4};

    int position = 0;
    PIDController cameraPIDController;
    VuforiaHelper vuforia;
    CryptoBoxColumnImageProcessor cryptoBoxFinder;

    int CRYPTO_COLUMN_TARGET_POSITION = 67;
    Bitmap curImage = null;
    ArrayList<Integer> coloredColumns;

    @Override
    public void runOpMode() {
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());

        }
        try{
            glyphLift = new ArialDepositor(hardwareMap);

        }
        catch(Exception e){
            Log.e("Error!","Glyph Lift: " + e.toString());
            throw new RuntimeException("Glyph Lift Creation Error! " + e.toString());
        }
        try{
            RAD = new JennyO1BRAD(hardwareMap);
        } catch (Exception e){
            Log.e("Error!", "Jenny RAD: " + e.toString());
            throw new RuntimeException("RAD Creation Error! " + e.toString());
        }
        try{
            jouster = new JewelJouster("jewelJouster",hardwareMap);
        } catch (Exception e){
            Log.e("Error!", "Jenny JewelJouster: " + e.toString());
            throw new RuntimeException("Jenny JewelJouster! " + e.toString());
        }
        try {
            glyphPicker = new JennyO1BGlyphPicker(hardwareMap);
        } catch (Exception e) {
            Log.e("Error!", "Glyph Picker: " + e.toString());
            throw new RuntimeException("Glyph Picker! " + e.toString());
        }
        sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
        flagController = new JennyFlagController(hardwareMap);
        leftJoystick = new JoystickHandler(gamepad1,JoystickHandler.LEFT_JOYSTICK);
        rightJoystick = new JoystickHandler(gamepad1,JoystickHandler.RIGHT_JOYSTICK);
        cameraPIDController = new PIDController(10.0/DESIRED_WIDTH,0,0);
        vuforia = new VuforiaHelper();
        cryptoBoxFinder = new CryptoBoxColumnImageProcessor(DESIRED_HEIGHT, DESIRED_WIDTH, CLOSE_UP_MIN_PERCENT_COLUMN_CHECK, CLOSE_UP_MIN_COLUMN_WIDTH, CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        jouster.setPosition(JewelJouster.EXTENDION_MODE.STORE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        jouster.setPosition(JewelJouster.EXTENDION_MODE.NEUTRAL);
        runtime.reset();
        boolean isSlowMode = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //DRIVE
            if(rightJoystick.magnitude() < .1){
                if(!isSlowMode)
                navigation.driveOnHeading((paralaxedControl)? (leftJoystick.angle() + 90)%360:leftJoystick.angle(), leftJoystick.magnitude() * HIGH_SPEED_IN_PER_SEC);
                else navigation.driveOnHeading((paralaxedControl)? (leftJoystick.angle() + 90)%360:leftJoystick.angle(), leftJoystick.magnitude() * SLOW_SPEED_IN_PER_SEC);
            }
            else {
                if(!isSlowMode) navigation.turn(rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x()));
                else navigation.turn(.1 * rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x()));
            }
            if(gamepad1.left_bumper && gamepad1.right_bumper){
                isSlowMode = !isSlowMode;
                while(gamepad1.left_bumper && gamepad1.right_bumper);
            }

            //GLYPH GRABBER
            if(gamepad1.a) glyphPicker.grab();
            else if(gamepad1.b) glyphPicker.spit();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.a) glyphPicker.grab();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.b) glyphPicker.spit();
            else glyphPicker.pause();

            //GLYPH LIFT
            if (!autoLiftPositionMode) {
                if (gamepad1.right_trigger > 0.1) {
                    if(!isSlowMode){
                        glyphLift.extend();
                    }
                    else{
                        glyphLift.slowExtend();
                    }
                }
                else if (gamepad1.right_bumper && !gamepad1.left_bumper && !glyphLift.isPressed()){
                    if(!isSlowMode){
                        glyphLift.retract();
                    }
                    else{
                        glyphLift.slowRetract();
                    }
                }
                else if (gamepad2.right_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper){
                    if(!isSlowMode){
                        glyphLift.extend();
                        if(glyphLift.isPressed()) glyphLift.setLiftPositionOffsetTicks(glyphLift.getLiftMotorPosition());
                    }
                    else {
                        glyphLift.slowExtend();
                        if(glyphLift.isPressed()) glyphLift.setLiftPositionOffsetTicks(glyphLift.getLiftMotorPosition());
                    }
                }

                else if (gamepad2.right_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper && !glyphLift.isPressed()){
                    if(!isSlowMode){
                        glyphLift.retract();
                    }
                    else{
                        glyphLift.slowRetract();
                    }
                }
                else if(!glyphLift.isPressed()) {
                    glyphLift.stopLift();
                }
                else
                    glyphLift.setLiftPower(0);
            } else {
                if (gamepad1.right_trigger > 0.1) {
                    position++;
                    while (gamepad1.right_trigger > 0.1) ;
                }
                if (gamepad1.right_bumper) {
                    position--;
                    while (gamepad1.right_bumper) ;
                }
                if (gamepad2.right_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                    position++;
                    while (gamepad2.right_trigger > 0.1) ;
                }

                if (gamepad2.right_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                    position--;
                    while (gamepad2.right_bumper) ;
                }

                if (position <= 0) position = 0;
                if (position >= liftPosition.length - 1) position = liftPosition.length - 1;
                glyphLift.goToGlyphLevel(liftPosition[position]);
                if(glyphLift.isPressed()) glyphLift.setLiftPositionOffsetTicks(glyphLift.getLiftMotorPosition());
            }
            if(gamepad2.back) {
                autoLiftPositionMode = !autoLiftPositionMode;
                while (gamepad1.back || gamepad2.back);
            }

            //GLYPH ROLLER
            if(!glyphLift.isPressed()) {
                glyphLift.setBeltPower(.25);
                if (gamepad1.left_trigger > 0.1) glyphLift.startBelt();
                else if (gamepad1.left_bumper && !gamepad1.right_bumper) glyphLift.reverseBelt();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.startBelt();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.reverseBelt();
                else
                    glyphLift.stopBelt();
            } else {
                glyphLift.setBeltPower(1);
                if (gamepad1.left_trigger > 0.1) glyphLift.startBelt();
                else if (gamepad1.left_bumper && !gamepad1.right_bumper) glyphLift.reverseBelt();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.startBelt();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.reverseBelt();
                else
                    glyphLift.stopBelt();
            }

            //RAD Extender
            if(gamepad2.dpad_right){
                RAD.extendRAD();
            }
            else if(gamepad2.dpad_left && !sensorTelemetry.isPressed(RAD_LIMIT)){
                RAD.retractRAD();
            }
            else {
                RAD.pauseRADExtender();
            }

            //RAD Grabber
            if(gamepad2.dpad_up){
                RAD.grabRelic();
            }
            else if(gamepad2.dpad_down){
                RAD.releaseRelic();
            }
            else {
                RAD.stopRelic();
            }

            //MISC
            if(gamepad1.back){
                paralaxedControl = !paralaxedControl;
                while (gamepad1.start);
            }
            if(gamepad1.right_stick_button){
                navigation.turnToHeading(NORTH, this);
            }
            if(gamepad1.left_stick_button){
                navigation.setOrientationOffset(360 - navigation.getOrientation());
            }
            if(gamepad1.x){
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                }
            }
            if(gamepad1.dpad_left){
                navigation.driveDistance(CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES, WEST, SLOW_SPEED_IN_PER_SEC, this);
                sleep(DEFAULT_SLEEP_DELAY_MILLIS);
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!centerOnCryptoBoxClosestToCenter(0,coloredColumns,WEST,EAST) && opModeIsActive() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                }
            }
            else if(gamepad1.dpad_right){
                navigation.driveDistance(CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES, EAST, SLOW_SPEED_IN_PER_SEC, this);
                sleep(DEFAULT_SLEEP_DELAY_MILLIS);
                curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                while(!centerOnCryptoBoxClosestToCenter(0,coloredColumns,EAST,WEST) && opModeIsActive() && !gamepad1.dpad_up && !gamepad1.a && !gamepad1.b){
                    curImage = vuforia.getImage(DESIRED_WIDTH, DESIRED_HEIGHT);
                    coloredColumns = cryptoBoxFinder.findColumns(curImage, false);
                }
            }
            if(gamepad2.left_stick_button){
                if(flagOn) flagController.pauseFlag();
                else flagController.startFlag();
                flagOn = !flagOn;
                while (gamepad2.start);
            }

            jouster.setPosition(JewelJouster.EXTENDION_MODE.NEUTRAL);
            telemetry.addData("lift tick", glyphLift.getLiftMotorPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        navigation.stopNavigation();
        glyphLift.stop();
        RAD.stop();
        sensorTelemetry.stopTelemetryLogging();
        glyphPicker.stop();
        flagController.stopFlag();
        jouster.stop();
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
