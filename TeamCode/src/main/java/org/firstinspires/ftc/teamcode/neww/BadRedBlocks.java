/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.neww;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.teamcode.neww.Robot.*;


@Autonomous(name = "Bad Red Blocks")
public class BadRedBlocks extends LinearOpMode {



    @Override
    public void runOpMode() {

        final double UP = 0; //servo up position
        final double DOWN = 1; //servo down position

        Robot.initMotors(this);


////////Program start////////////////////////////////////////////////////////////////////////
        waitForStart();

        forward(8.3,1);

        ////////run camera///////////////////////////////////////////////////////////////////////
        final int TIME_LIMIT = 600; //camera timeout time in ms
        boolean scan1 = false;
        boolean scan2 = false;

        //init camera stuff
        initVuforia(this);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(this);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }


        //scan 1
        long time = System.currentTimeMillis(); //activates timer
        if (opModeIsActive()) {
            while (opModeIsActive()) { //maybe add time limit to this as well
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals("Skystone")) {
                                scan1 = true;
                                break;
                            }
                        }
                    }
                }
                //breaks when reached time limit
                if ((System.currentTimeMillis()-time)>TIME_LIMIT){
                    telemetry.addLine("time limit reached");
                    break;
                }
            }
        }

        left(14,.5);//strafe to scan 2

        //scan 2
        time = System.currentTimeMillis(); //activates timer
        if (opModeIsActive()) {
            while (opModeIsActive()) { //maybe add time limit to this as well
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals("Skystone") && recognition.getLeft() < 200) {
                                telemetry.addData("left", recognition.getLeft());
                                scan2 = true;
                                break;
                            }
                        }
                    }
                }
                //breaks when reached time limit
                if ((System.currentTimeMillis()-time)>TIME_LIMIT){
                    telemetry.addLine("time limit reached");
                    break;
                }
            }
        }


        ////////camera finished///////////////////////////////////////////////////////////////////
        telemetry.addData("scan1 ", scan1);
        telemetry.addData("scan2", scan2);
        telemetry.update();

        //calculates skystone position
        if (scan1 && scan2){
            //blockposition 1
            right(11.2,.5);
        }
        else if (scan1){
            //blockpos 0
            right(22,.5);
        }
        else {
            //blockpos 2

        }



        //go and grab skystone
        forward(19,1);
        forward(3,.5);
        blockgrabRed.setPosition(DOWN); //block grabbed
        sleep(750);
        back(15,1);

        //return to centered position
        if (scan1 && scan2){
            //blockpos1
            right(8.8,.5);
        }
        else if (scan1){
            //blockpos0
        }
        else {
            //blockpos 2
            right(22,.5);
        }

        //////////aligned with block in same position every time/////////////////////////////////

        clock(DEG90,1);
        forward(40,1);
        blockgrabRed.setPosition(UP);
        sleep(500);
        back(70,1);
        counter(DEG90,1);
        left(12,.4); //strafe into wall
        sleep(100);

        if (scan1 && scan2){
            //blockposition 1
            right(10,.5);
        }
        else if (scan1){
            //blockpos 0
            right(18,.5);
        }
        else {
            //blockpos 2

        }

        //go and grab skystone 2
        forward(12,1);
        forward(2,.5);
        blockgrabRed.setPosition(DOWN); //block 2 grabbed
        sleep(500);
        back(12,1);

        //return to centered position
        if (scan1 && scan2){
            //blockpos1
            right(8,.5);
        }
        else if (scan1){
            //blockpos0
        }
        else {
            //blockpos 2
            right(18,.5);
        }

        clock(DEG90,1);
        forward(59,1);
        blockgrabRed.setPosition(UP);
        sleep(200);
        back(13,1);


    }


}