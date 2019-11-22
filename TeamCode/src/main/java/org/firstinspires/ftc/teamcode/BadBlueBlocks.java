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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Bad Blue Blocks")
public class BadBlueBlocks extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "Af+w9bD/////AAABmRAQ4PuIw0sdh/byTqA444Jo20dWZi+e6ZOHAsur5coDrAZ2k0LohL3kbR4Toa0yyYSOgCLXvIlm1ne8ZdjMTbVzEvsG0cfOrr3zHqV94jRsRa+sfeyeiTlEZFwz22a3eJX0CI5ga1JRfVaY8f3h1Kf6oxZbSF9rHraCjV1+egDARh4QmNWWHS0DKZi64hLwAu7P4NWsFFHN95eRdz3P7t4eQIZwX8vtAedGEkTM3V3tO8aYFcQ1MEPgHL+B+CTleFScXD8gjoMjCrVeZ4qNfkVda3bR3IUZSp8XaoL9GMy5irmgLJBNeo/H9qq3yFelSUIQMCZ1awAecpV6oHS3yAeaL8J+Bwe2/ZplShgiGPzS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    DcMotor rightfront;
    DcMotor leftfront;
    DcMotor leftback;
    DcMotor rightback;
    DcMotor armboi;
    DcMotor slide;
    Servo blockgrab;
    Servo grabright;
    Servo grableft;
    final int TICKS_PER_INCH = 90;
    final double UP = 1; //servo up position
    final double DOWN = 0; //servo down position
    final double DEG90 = 21.5;//90 degree turn distance
    int blockposition = 69;


    @Override
    public void runOpMode() {



        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        blockgrab = hardwareMap.get(Servo.class, "servo2");
        armboi = hardwareMap.get(DcMotor.class, "armboi");
        slide = hardwareMap.get(DcMotor.class,"armboislides");
        grabright = hardwareMap.get(Servo.class, "servo0");
        grableft = hardwareMap.get(Servo.class, "servo1");
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        armboi.setDirection(DcMotor.Direction.REVERSE);
        armboi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

////////Program start////////////////////////////////////////////////////////////////////////
        waitForStart();

        forward(8.3,1);

        /*
        resetEncoders();
        SetPower(.3, -.3, -.3, .3); //begin strafing right
        */


        ////////run camera///////////////////////////////////////////////////////////////////////
        final int TIME_LIMIT = 1500; //camera timeout time in ms
        boolean scan1 = false;
        boolean scan2 = false;

        //init camera stuff
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
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

        right(14,.5);//strafe to scan 2

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
                            if (recognition.getLabel().equals("Skystone")) {
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


        //calculates skystone position
        if (scan1 && scan2){
            //blockposition 1
            blockposition = 1;
            left(11.2,.5);
        }
        else if (scan1){
            //blockpos 0
            left(20,.5);
        }
        else {
            //blockpos 2

        }
        telemetry.addData("blockposition ", blockposition);
        telemetry.update();

        //go and grab skystone
        forward(19,1);
        blockgrab.setPosition(DOWN); //block grabbed
        sleep(750);
        back(15,1);

        //return to centered position
        if (scan1 && scan2){
            //blockpos1
            left(8.8,.5);
        }
        else if (scan1){
            //blockpos0
        }
        else {
            //blockpos 2
            left(20,.5);
        }

        //////////aligned with block in same position every time/////////////////////////////////

        counter(DEG90,1);
        forward(50,1);
        blockgrab.setPosition(UP);
        sleep(500);
        back(50,1);
        clock(DEG90,1);




    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.3;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void forward(double dist, double speed) {
        move(speed, dist, dist, dist, dist);
    }
    public void back(double dist, double speed) {
        move(speed, -dist, -dist, -dist, -dist);
    }
    public void counter(double dist, double speed) {
        move(speed, -dist, -dist, dist, dist);
    }
    public void clock(double dist, double speed){ move(speed, dist, dist, -dist, -dist); }
    public void right(double dist, double speed) { move(speed, -dist, dist, dist, -dist); }
    public void left(double dist, double speed) { move(speed, dist, -dist, -dist, dist); }


    public void resetEncoders(){
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double[] encoderValues(){
        double[] encoderArray = {rightfront.getCurrentPosition(), rightback.getCurrentPosition(), leftfront.getCurrentPosition(), leftback.getCurrentPosition()};
        return encoderArray;
    }

    public void move(double speed, double distRF, double distRB, double distLF, double distLB) {
        resetEncoders();

        int posRF = (int)(distRF * TICKS_PER_INCH);
        int posRB = (int)(distRB * TICKS_PER_INCH);
        int posLF = (int)(distLF * TICKS_PER_INCH);
        int posLB = (int)(distLB * TICKS_PER_INCH);


        rightfront.setTargetPosition(posRF);
        rightback.setTargetPosition(posRB);
        leftfront.setTargetPosition(posLF);
        leftback.setTargetPosition(posLB);

        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightfront.setPower(speed);
        rightback.setPower(speed);
        leftfront.setPower(speed);
        leftback.setPower(speed);

        while(rightfront.isBusy() && rightback.isBusy() && leftfront.isBusy() && leftback.isBusy()) {

        }

        rightfront.setPower(0);
        rightback.setPower(0);
        leftfront.setPower(0);
        leftback.setPower(0);
    }

    public void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);
    }

}
