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
    HeadingVector v3;
    HeadingVector v4;
    HeadingVector v5;
    HeadingVector v6;
    HeadingVector v7;
    HeadingVector v8;

    @Override
    public void runOpMode() {
        v1 = new HeadingVector(0, 5);
        v2 = new HeadingVector(3.5355339059327378, 3.5355339059327378);
        v3 = new HeadingVector(5, 0);
        v4 = new HeadingVector(3.5355339059327378, -3.5355339059327378);
        v5 = new HeadingVector(0, -5);
        v6 = new HeadingVector(-3.5355339059327378, -3.5355339059327378);
        v7 = new HeadingVector(-5, 0);
        v8 = new HeadingVector(-3.5355339059327378, 3.5355339059327378);
//        v1.calculateVector(0, 5);
//        v2.calculateVector(45, 5);
//        v3.calculateVector(90, 5);
//        v4.calculateVector(135, 5);
//        v5.calculateVector(180, 5);
//        v6.calculateVector(225, 5);
//        v7.calculateVector(270, 5);
//        v8.calculateVector(315, 5);
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
        Log.d("V3 Magnitude", Double.toString(v3.getMagnitude()));
        Log.d("V3 Heading", Double.toString(v3.getHeading()));
        Log.d("V3 x", Double.toString(v3.x()));
        Log.d("V3 y", Double.toString(v3.y()));
        Log.d("V4 Magnitude", Double.toString(v4.getMagnitude()));
        Log.d("V4 Heading", Double.toString(v4.getHeading()));
        Log.d("V4 x", Double.toString(v4.x()));
        Log.d("V4 y", Double.toString(v4.y()));
        Log.d("V5 Magnitude", Double.toString(v5.getMagnitude()));
        Log.d("V5 Heading", Double.toString(v5.getHeading()));
        Log.d("V5 x", Double.toString(v5.x()));
        Log.d("V5 y", Double.toString(v5.y()));
        Log.d("V6 Magnitude", Double.toString(v6.getMagnitude()));
        Log.d("V6 Heading", Double.toString(v6.getHeading()));
        Log.d("V6 x", Double.toString(v6.x()));
        Log.d("V6 y", Double.toString(v6.y()));
        Log.d("V7 Magnitude", Double.toString(v7.getMagnitude()));
        Log.d("V7 Heading", Double.toString(v7.getHeading()));
        Log.d("V7 x", Double.toString(v7.x()));
        Log.d("V7 y", Double.toString(v7.y()));
        Log.d("V8 Magnitude", Double.toString(v8.getMagnitude()));
        Log.d("V8 Heading", Double.toString(v8.getHeading()));
        Log.d("V8 x", Double.toString(v8.x()));
        Log.d("V8 y", Double.toString(v8.y()));
        // run until the end of the match (driver presses STOP)
    }
}
