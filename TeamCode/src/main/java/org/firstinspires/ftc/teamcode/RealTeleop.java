package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "teleoperation")
public class RealTeleop extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Af+w9bD/////AAABmRAQ4PuIw0sdh/byTqA444Jo20dWZi+e6ZOHAsur5coDrAZ2k0LohL3kbR4Toa0yyYSOgCLXvIlm1ne8ZdjMTbVzEvsG0cfOrr3zHqV94jRsRa+sfeyeiTlEZFwz22a3eJX0CI5ga1JRfVaY8f3h1Kf6oxZbSF9rHraCjV1+egDARh4QmNWWHS0DKZi64hLwAu7P4NWsFFHN95eRdz3P7t4eQIZwX8vtAedGEkTM3V3tO8aYFcQ1MEPgHL+B+CTleFScXD8gjoMjCrVeZ4qNfkVda3bR3IUZSp8XaoL9GMy5irmgLJBNeo/H9qq3yFelSUIQMCZ1awAecpV6oHS3yAeaL8J+Bwe2/ZplShgiGPzS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    DcMotor armboi;
    DcMotor rightfront;
    DcMotor leftfront;
    DcMotor leftback;
    DcMotor rightback;
    DcMotor slide;
    Servo grabright;
    Servo grableft;
    Servo blockgrab;
    Servo grableft2;
    Servo grabright2;

    ArrayList<String> info = new ArrayList<>();
    String dir = "";

    int slidepos = 0;
    int armpos = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }

        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        armboi = hardwareMap.get(DcMotor.class, "armboi");
        slide = hardwareMap.get(DcMotor.class,"armboislides");

        //autonomous arm
        blockgrab = hardwareMap.get(Servo.class, "servo2");
        //bottom two grabbers
        grabright = hardwareMap.get(Servo.class, "servo0");
        grableft = hardwareMap.get(Servo.class, "servo1");
        //top two grabbers
        grabright2 = hardwareMap.get(Servo.class, "servo4");

        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        armboi.setDirection(DcMotor.Direction.REVERSE);


        resetMotors();

        waitForStart();

        //servo starting positions
        grableft.setPosition(.9);
        grabright.setPosition(0);


        double getLeftPos = 0;
        double getRightPos = 0;

        while(opModeIsActive()) {
            boolean a1 = gamepad1.a;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;
            boolean leftBump1 = gamepad1.left_bumper;
            boolean rightBump1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y / 2;
            double LStickX  =  -gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x / 2;

            double LTrigger1 = -gamepad1.left_trigger / 2;
            double RTrigger1 = -gamepad1.right_trigger / 2;


            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double LTrigger2 = gamepad2.left_trigger;
            double RTrigger2 = gamepad2.right_trigger;
            boolean LBumper2 = gamepad2.left_bumper;
            boolean RBumper2 = gamepad2.right_bumper;

            double RStickY2 = -gamepad2.right_stick_y;
            double RStickX2 = gamepad2.right_stick_x;
            double LStickY2 = -gamepad2.left_stick_y;

            boolean dpadUp1 = gamepad1.dpad_up;
            boolean dpadRight1 = gamepad1.dpad_right;
            boolean dpadLeft1 = gamepad1.dpad_left;
            boolean dpadDown1 = gamepad1.dpad_down;

            boolean dpadUP2 = gamepad2.dpad_up;
            boolean dpadDOWN2 =gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;



            //vuforia output
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // step through the list of recognitions and display boundary info
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone")) {
                            getLeftPos = recognition.getLeft();
                            getRightPos = recognition.getRight();
                        }
                    }
                }
            }

            telemetry.addData("getLeft pos: ", getLeftPos);
            telemetry.addData("getRight pos: ", getRightPos);


            //driving
            if (Math.abs(LStickY) > 0) {
                SetPower(LStickY, LStickY, LStickY, LStickY);
            }
            else if(Math.abs(RStickX) > 0) {
                SetPower(RStickX, -RStickX, RStickX, -RStickX);
            }
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-LTrigger1, LTrigger1, LTrigger1, -LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(RTrigger1, -RTrigger1, -RTrigger1, RTrigger1);
            }
            else if (leftBump1) {
                SetPower(.3, -.3, .3, -.3);
            }
            else if (rightBump1) {
                SetPower(-.3, .3, -.3, .3);
            }
            else {
                SetPower(0, 0, 0, 0);
            }


            //d pad fine tuned driving
            if(dpadUp1){
                SetPower(-.3, -.3, -.3, -.3);
            }
            else if(dpadRight1){
                SetPower(-.5, .5, .5, -.5);
            }
            else if(dpadLeft1){
                SetPower(.5, -.5, -.5, .5);
            }
            else if(dpadDown1){
                SetPower(.3, .3, .3, .3);
            }


            //autonomous servo
            if (x1) { //down
                blockgrab.setPosition(.5);
            }
            if (y1) { //up (starting)
                blockgrab.setPosition(1);
            }


            if (a1) {
                if (leftfront.getCurrentPosition() > 0 && rightfront.getCurrentPosition() > 0 &&
                        leftback.getCurrentPosition() > 0 && rightback.getCurrentPosition() > 0) {
                    dir = "forward  ";
                }
                else if (leftfront.getCurrentPosition() < 0 && rightfront.getCurrentPosition() < 0 &&
                        leftback.getCurrentPosition() < 0 && rightback.getCurrentPosition() < 0) {
                    dir = "back  ";
                }
                else if (leftfront.getCurrentPosition() > 0 && rightfront.getCurrentPosition() < 0 &&
                        leftback.getCurrentPosition() > 0 && rightback.getCurrentPosition() < 0) {
                    dir = "clock  ";
                }
                else if (leftfront.getCurrentPosition() < 0 && rightfront.getCurrentPosition() > 0 &&
                        leftback.getCurrentPosition() < 0 && rightback.getCurrentPosition() > 0) {
                    dir = "counter  ";
                }
                else if (leftfront.getCurrentPosition() > 0 && rightfront.getCurrentPosition() < 0 &&
                        leftback.getCurrentPosition() < 0 && rightback.getCurrentPosition() > 0) {
                    dir = "right  ";
                }
                else if (leftfront.getCurrentPosition() < 0 && rightfront.getCurrentPosition() > 0 &&
                        leftback.getCurrentPosition() > 0 && rightback.getCurrentPosition() < 0) {
                    dir = "left  ";
                }

                info.add(dir + leftfront.getCurrentPosition() + " " + rightfront.getCurrentPosition()
                        + " " + leftback.getCurrentPosition() + " " + rightback.getCurrentPosition());

                resetMotors();
            }


            telemetry.addData("left front", leftfront.getCurrentPosition());
            telemetry.addData("right front", rightfront.getCurrentPosition());
            telemetry.addData("left back", leftback.getCurrentPosition());
            telemetry.addData("right back", rightback.getCurrentPosition());
            telemetry.addLine("-------------------------");


            if (!info.isEmpty()) {
                for (String data: info) {
                    telemetry.addLine(data);
                }
            }

            telemetry.update();
        }

        //threadSlide.interrupt();
        //threadArm.interrupt();

    }



    public void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        leftfront.setPower(-LFPower);
        leftback.setPower(-LBPower);
        rightfront.setPower(-RFPower);
        rightback.setPower(-RBPower);
    }

    public void resetMotors() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

}


