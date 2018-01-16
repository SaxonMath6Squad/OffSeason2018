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

import Actions.ArialDepositor;
import Actions.JewelJouster;
import DriveEngine.JennyNavigation;

import SensorHandlers.JennySensorTelemetry;

import static Autonomous.REVColorDistanceSensorController.color.BLUE;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.ADJUSTING_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SOUTH;

/*
    An opmode to test knocking off the correct jewel
 */
@Autonomous(name="Jewel Joust Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class JewelJoustTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    ArialDepositor glyphSystem;
    JennySensorTelemetry sensorTelemetry;
    JewelJouster jewelJouster;
    //ImuHandler imuHandler;
    @Override
    public void runOpMode() {
        //imuHandler = new ImuHandler("imu", hardwareMap);
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
            glyphSystem = new ArialDepositor(hardwareMap);
            sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
            jewelJouster = new JewelJouster("jewelJouster", hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //sensorTelemetry.jewelJoust.setPosition(JEWEL_JOUST_ACTIVE_POSITION);
        jewelJouster.setPosition(JewelJouster.EXTENDION_MODE.READ);
        sleep(500);
        if(jewelJouster.getJewelColor() == BLUE){
            navigation.turnToHeading(15, this);
            jewelJouster.setPosition(JewelJouster.EXTENDION_MODE.HIT);
            sleep(DEFAULT_SLEEP_DELAY_MILLIS);
            navigation.driveDistance(3, SOUTH, ADJUSTING_SPEED_IN_PER_SEC, this);
            sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        }
        else {
            navigation.driveDistance(1.5, SOUTH, ADJUSTING_SPEED_IN_PER_SEC, this);
            sleep(DEFAULT_SLEEP_DELAY_MILLIS);
            jewelJouster.setPosition(JewelJouster.EXTENDION_MODE.HIT);
            navigation.driveDistance(2, NORTH, ADJUSTING_SPEED_IN_PER_SEC, this);
            sleep(DEFAULT_SLEEP_DELAY_MILLIS);
        }

        navigation.stopNavigation();
//        glyphSystem.stopNavigation();
    }
}
