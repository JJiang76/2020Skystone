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

package org.firstinspires.ftc.teamcode.unused;

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

@Disabled
@Autonomous(name = "TF auto blue blocks")
public class TFautoBlueBlocks extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    DcMotor rightfront;
    DcMotor leftfront;
    DcMotor leftback;
    DcMotor rightback;
    DcMotor armboi;
    DcMotor slide;
    Servo blockgrabBlue;
    Servo grabright;
    Servo grableft;
    Servo cap;
    int blockPosition = 69;
    final int TICKS_PER_INCH = 90;
    private static final String VUFORIA_KEY =
            "Af+w9bD/////AAABmRAQ4PuIw0sdh/byTqA444Jo20dWZi+e6ZOHAsur5coDrAZ2k0LohL3kbR4Toa0yyYSOgCLXvIlm1ne8ZdjMTbVzEvsG0cfOrr3zHqV94jRsRa+sfeyeiTlEZFwz22a3eJX0CI5ga1JRfVaY8f3h1Kf6oxZbSF9rHraCjV1+egDARh4QmNWWHS0DKZi64hLwAu7P4NWsFFHN95eRdz3P7t4eQIZwX8vtAedGEkTM3V3tO8aYFcQ1MEPgHL+B+CTleFScXD8gjoMjCrVeZ4qNfkVda3bR3IUZSp8XaoL9GMy5irmgLJBNeo/H9qq3yFelSUIQMCZ1awAecpV6oHS3yAeaL8J+Bwe2/ZplShgiGPzS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        blockgrabBlue = hardwareMap.get(Servo.class, "servo2");
        armboi = hardwareMap.get(DcMotor.class, "armboi");
        slide = hardwareMap.get(DcMotor.class,"armboislides");

        grabright = hardwareMap.get(Servo.class, "servo0");
        grableft = hardwareMap.get(Servo.class, "servo1");
        cap = hardwareMap.get(Servo.class, "cap");

        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        armboi.setDirection(DcMotor.Direction.REVERSE);

        armboi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        cap.setPosition(0);

        long time = System.currentTimeMillis();
        if (opModeIsActive()) {
            while (opModeIsActive() && blockPosition > 3) { //maybe add time limit to this as well
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {

                            if (recognition.getLabel().equals("Skystone")) {
                                if (recognition.getLeft() < 40) {
                                    blockPosition = 0;
                                    break;
                                }
                                else if(recognition.getRight() > 350){
                                    if (recognition.getLeft() < 220){
                                        blockPosition =1;
                                        telemetry.addLine("here");
                                    }
                                    else {
                                        blockPosition = 2;
                                    }
                                    break;
                                }
                                else{
                                    blockPosition = 1;
                                }
                            }
                            else if ((System.currentTimeMillis()-time)>2000){
                                blockPosition = 1;
                                telemetry.addLine("time limit reached");

                                break;
                            }
                        }

                    } else {
                        blockPosition = 1;
                        telemetry.addLine("no objects detected");
                    }
                }

            }
        }

        telemetry.addData("block position out loop", blockPosition);
        telemetry.update();

        blockgrabBlue.setPosition(1);

        if (blockPosition == 0) { //left skystone
            left(15,1);
            forward(30,.75);
            blockgrabBlue.setPosition(0);
            sleep(750);
            back(15,1);
            counter(22, 1);
        }
        else if (blockPosition == 1) { //mid skystone
            //forward
            left(7,1);
            forward(30,.75);
            blockgrabBlue.setPosition(0);
            sleep(750);
            back(15,1);
            counter(22, 1);
        }
        else { //right skystone
            //right 7in
            right(8,1);
            //forward
            forward(30,.75);
            blockgrabBlue.setPosition(0);
            sleep(750);
            back(15,1);
            counter(22, 1);
        }

        //all 3 possibilities are alliged the same

        forward(50, 1);
        if (blockPosition == 2) {
            forward(5, 1);
        }
        blockgrabBlue.setPosition(1);
        back(71.5, 1);
        clock(21.5,1);
        if (blockPosition == 2) {
            right(7, 1);
        }

        forward(17,.75);
        blockgrabBlue.setPosition(0);//block two grabbed
        sleep(750);
        back(14,1);
        counter(22,1);
        forward(75,1);
        blockgrabBlue.setPosition(1);
        sleep(250);
        back(18,1);

        if (blockPosition == 0) {
            back(7, 1);
        }

        right(10, 1);









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
        tfodParameters.minimumConfidence = 0.4;
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




    public void move(double speed, double distRF, double distRB, double distLF, double distLB) {
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
