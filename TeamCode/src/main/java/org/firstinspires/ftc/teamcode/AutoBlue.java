package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "blue auto")
public class AutoBlue extends LinearOpMode {
    DcMotor rightfront;
    DcMotor leftfront;
    DcMotor leftback;
    DcMotor rightback;
    DcMotor armboi;
    Servo blockgrab;
    Servo grabright;
    Servo grableft;

    final int TICKS_PER_INCH = 90;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("init");
        telemetry.update();

        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        blockgrab = hardwareMap.get(Servo.class, "servo2");
        armboi = hardwareMap.get(DcMotor.class, "armboi");

        grabright = hardwareMap.get(Servo.class, "servo0");
        grableft = hardwareMap.get(Servo.class, "servo1");

        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        armboi.setDirection(DcMotor.Direction.REVERSE);

        armboi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        blockgrab.setPosition(1);
        forward(29, 1); //approaches  a block
        blockgrab.setPosition(.25);
        sleep(500);
        //block grabbed


        back(14,1);
        clock(22, 1);
        back(50, 1);
        blockgrab.setPosition(.5);
        back(27, 1);
        //block dropped off and servo hopefully out of way

        clock(22,1);
        blockgrab.setPosition(1);
        back(16,.75); //approaches mat
        blockgrab.setPosition(.5);
        sleep(500);

        //opens servos and lowers arm boi onto mat
        grableft.setPosition(.463);
        grabright.setPosition(.435);
        sleep(500);
        armboi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armboi.setTargetPosition(1060);
        armboi.setPower(.5);
        armboi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armboi.isBusy()) {} //use a sleep instead of while so program will continue regardless
        armboi.setPower(0);

        blockgrab.setPosition(1);
        forward(37,1); //backed up against wall(with hopefully mat)

        //raise armboi back off of mat
        blockgrab.setPosition(.7);//puts blockgrab against wall
        armboi.setTargetPosition(2);
        armboi.setPower(.5);
        armboi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armboi.isBusy()) {} //use a sleep instead of while so program will continue regardless
        armboi.setPower(0);
        //servos default position
        grableft.setPosition(.9);
        grabright.setPosition(0);
        blockgrab.setPosition(1);

        left(107,1);//go get another cube

        back(10,1);
        clock(42, 1);//turn around
        forward(19,1);
        right(1.5, 1);
        blockgrab.setPosition(.25);
        sleep(500);
        //another block grabbed

        back(16,1);
        counter(22, 1);
        forward(60, 1); //drives to line
        blockgrab.setPosition(1);
        back(19, 1);

    }


    //90 degrees = 22.22 in
    //1 inch lateral = 100 ticks

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
