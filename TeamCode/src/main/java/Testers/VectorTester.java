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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import Autonomous.HeadingVector;

@TeleOp(name="HeadingVector test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class VectorTester extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    HeadingVector v1;
    HeadingVector v2;

    @Override
    public void runOpMode() {
        v1 = new HeadingVector(1, -5);
        v2 = new HeadingVector(4, 0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        Log.d("V1 Magnitude", Double.toString(v1.getMagnitude()));
        Log.d("V1 Heading", Double.toString(v1.getHeading()));
        Log.d("V1 x", Double.toString(v1.x()));
        Log.d("V1 y", Double.toString(v1.y()));
        Log.d("V2 Magnitude", Double.toString(v2.getMagnitude()));
        Log.d("V2 Heading", Double.toString(v2.getHeading()));
        Log.d("V2 x", Double.toString(v2.x()));
        Log.d("V2 y", Double.toString(v2.y()));
        Log.d("V1 + V2", Double.toString(v1.addVector(v2).x()) + ", " + Double.toString(v1.addVector(v2).y()));
        Log.d("V1 + V2 Heading", Double.toString(v1.addVector(v2).getHeading()));
        Log.d("V1 + V2 Magnitude", Double.toString(v1.addVector(v2).getMagnitude()));
        Log.d("V1 - V2", Double.toString(v1.subtractVector(v2).x()) + ", " + Double.toString(v1.subtractVector(v2).y()));
        Log.d("V1 - V2 Heading", Double.toString(v1.subtractVector(v2).getHeading()));
        Log.d("V1 - V2 Magnitude", Double.toString(v1.subtractVector(v2).getMagnitude()));
        // run until the end of the match (driver presses STOP)
    }
}
