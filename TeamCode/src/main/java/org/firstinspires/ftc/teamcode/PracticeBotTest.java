package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PracticeBotTest", group="Sensor")
public class PracticeBotTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private double power = 0.5;
    private boolean shooterOn = false;
    private DcMotor shooter;
    private Servo shooterRight;
    private Servo shooterLeft;
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterLeft = hardwareMap.get(Servo.class, "shooter_left");
        shooterRight = hardwareMap.get(Servo.class, "shooter_right");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLeft.setDirection(Servo.Direction.FORWARD);
        shooterRight.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        double max;

        double yaw;
        double lateral;

        double rightDrivePower;
        double leftDrivePower;

        while (opModeIsActive()) {

            if (gamepad1.dpadDownWasPressed()) {
                power -= 0.01;
            } else if (gamepad1.dpadUpWasPressed()) {
                power += 0.01;
            } else if (gamepad1.dpadLeftWasPressed()) {
                power -= 0.05;
            } else if (gamepad1.dpadRightWasPressed()) {
                power += 0.05;
            }

            if (gamepad1.aWasPressed()) {
                shooterOn = !shooterOn;
            }

            shooter.setPower(shooterOn ? power : 0);

            if (gamepad1.x) {
                shooterLeft.setPosition(1);
                shooterRight.setPosition(1);
            } else {
                shooterLeft.setPosition(0.5);
                shooterRight.setPosition(0.5);
            }

            yaw = -gamepad1.right_stick_x;
            lateral = gamepad1.left_stick_y;

            leftDrivePower = lateral + yaw;
            rightDrivePower = lateral - yaw;

            max = Math.max(Math.abs(leftDrivePower), Math.abs(rightDrivePower));

            if (max > 1.0) {
                leftDrivePower /= max;
                rightDrivePower /= max;
            }

            leftDrive.setPower(leftDrivePower);
            rightDrive.setPower(rightDrivePower);


            telemetry.addData("Power", power);
            telemetry.addData("Elapsed Time", runtime.milliseconds() / 1000);
            telemetry.update();
        }
    }
}