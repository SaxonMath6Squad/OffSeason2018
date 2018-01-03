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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import Autonomous.Location;
import DriveEngine.JennyNavigation;

import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;

/*
    An opmode to test the newDriveOnHeading function
 */
@Autonomous(name="New Drive On Heading Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class NewDriveOnHeadingTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Location start = new Location(0, 0);
    Location second = new Location(-12, 12);
    Location third = new Location(0, 24);
    Location fourth = new Location(12, 12);
    JennyNavigation navigation;
    //ImuHandler imuHandler;
    @Override
    public void runOpMode() {
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
        } catch (Exception e) {
            Log.e("Error! ", "Navigation Creation Failed!");
            throw new RuntimeException(e.toString());
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
//            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(0, 10, 10, this);
//            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(90, 10, 10, this);
//            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(180, 10, 10, this);
//            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(270, 10, 10, this);
//            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(45, 10, 10, this);
//            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(135, 10, 10, this);
//            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(225, 10, 10, this);
//            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(315, 10, 10, this);
//            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(20, 10, 10, this);
//            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(40, 10, 10, this);
//            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(60, 10, 10, this);
//            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(80, 10, 10, this);
//            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(100, 10, 10, this);
//            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(120, 10, 10, this);
//            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(140, 10, 10, this);
//            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(160, 10, 10, this);
//            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(200, 10, 10, this);
//            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(220, 10, 10, this);
//            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(240, 10, 10, this);
//            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(260, 10, 10, this);
//            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(280, 10, 10, this);
//            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(300, 10, 10, this);
//            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(320, 10, 10, this);
//            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(340, 10, 10, this);
//            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(22.5, 10, 10, this);
//            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(67.5, 10, 10, this);
//            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(112.5, 10, 10, this);
//            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(157.5, 10, 10, this);
            if(gamepad1.dpad_up) navigation.newDriveOnHeadingIMU(202.5, 10, 10, this);
            else if(gamepad1.dpad_right) navigation.newDriveOnHeadingIMU(247.5, 10, 10, this);
            else if(gamepad1.dpad_down) navigation.newDriveOnHeadingIMU(292.5, 10, 10, this);
            else if(gamepad1.dpad_left) navigation.newDriveOnHeadingIMU(337.5, 10, 10, this);
            else navigation.brake();
        }
        navigation.stopNavigation();
    }
}
