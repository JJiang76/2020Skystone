package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "phone name")
public class autotest extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "config name");
        motor2 = hardwareMap.get(DcMotor.class, "config name");

        waitForStart();

        motor1.setPower(1);
        motor2.setPower(1);
        sleep(2500);
        motor1.setPower(0);
        motor2.setPower(0);
    }
}
