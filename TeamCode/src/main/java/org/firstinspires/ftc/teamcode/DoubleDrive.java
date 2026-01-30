/*
TELEOP CONTROLS

Gamepad 1:

DURING INIT:

X - change team to blue
B - change team to red

DURING PLAY:

ERLIS:
Left stick - translate robot
Right stick - rotate robot
Dpad Up - high power
Dpad Down - low power
Dpad Left - low speed for parking
Dpad Right - high speed for driving
Left bumper - power down by .05
Right bumper - power up by .05
A - hold to turn robot to shooting angle

BRODY:
Dpad Up - Toggle flywheel
Dpad Down - Toggle intake
Dpad Left - Toggle feed servos
Right Bumper - Shoot ball
X - Cycle sorter left
B - Cycle sorter right
Y - Home sorter


Expansion -
Motor 0 front right
motor 1 back right
motor 2 shooter right
motor 3 intake

servo 0 flick
i2c 0 pinpoint

Control -
motor 0 shooter left
motor 1 front left
motor 2 back left
servo 1 sorter
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Sorter;


@TeleOp(name="DoubleDrive", group="A")
public class DoubleDrive extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private double power = 0.75;
    private final double MIN_POWER = 0.75;
    private final double MAX_POWER = 0.9;
    private double speed = 1;
    private final double LOW_SPEED = 0.6;
    private final double HIGH_SPEED = 1;
    private final double FINGER_HOME = 1;
    private final double FINGER_ACTIVE = 0.1;
    private boolean intakeSpinning = false;
    /*
    private boolean feedSpinning = false;
    */
    private boolean shooterSpinning = false;
    private boolean blueTeam = true; // TRUE = BLUE TEAM, FALSE = RED TEAM
    private Sorter sorter = new Sorter();
    private Shooter shooter = new Shooter();
    private DcMotor intake;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private Limelight3A limelight;
    private HuskyLens huskyLens;
    private GoBildaPinpointDriver pinpoint;

    public int[] readObelisk() {
        int[] pattern = {0,0,0};
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
        }
        return pattern;
    }

    public double getAprilTagPosX() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                return result.getTx() - 10;
            }
        }
        return 0;
    }

    public double getTargetYaw(Pose2D bin) {
        Pose2D robotPos = pinpoint.getPosition();
        double distX = bin.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH);
        double distY = bin.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH);
        return Math.tan(distY/distX);
    }

    @Override
    public void init() {
        shooter.init(hardwareMap);
        sorter.init(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-73.51,-165.8,DistanceUnit.MM);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();

        limelight.pipelineSwitch(0);
    }
    @Override
    public void init_loop() {
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", blueTeam ? "Blue" : "Red");
        telemetry.addData("Marination", runtime.toString());
        telemetry.update();

        if (gamepad1.x) {
            blueTeam = true;
        } else if (gamepad1.b) {
            blueTeam = false;
        }

        //binPos = blueTeam ? new Pose2D(DistanceUnit.INCH,72,0,AngleUnit.DEGREES,0) : new Pose2D(DistanceUnit.INCH,72,144,AngleUnit.DEGREES,0);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double max;

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        if (gamepad1.a) {
            yaw = getAprilTagPosX()/160;
        }

        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftPower *= speed;
        frontRightPower *= speed;
        backLeftPower *= speed;
        backRightPower *= speed;

        // CHANGE SHOOTER POWER MANUALLY
        /*
        if (gamepad1.dpadDownWasPressed()) {
            power -= 0.01;
        } else if (gamepad1.dpadUpWasPressed()) {
            power += 0.01;
        } else if (gamepad1.dpadLeftWasPressed()) {
            power -= 0.05;
        } else if (gamepad1.dpadRightWasPressed()) {
            power += 0.05;
        } */

        // Set power

        if (gamepad1.dpadUpWasPressed()) {
            power = MAX_POWER;
        } else if (gamepad1.dpadDownWasPressed()) {
            power = MIN_POWER;
        }

        // Set power
        if (gamepad1.dpadRightWasPressed()) {
            speed = HIGH_SPEED;
        } else if (gamepad1.dpadLeftWasPressed()) {
            speed = LOW_SPEED;
        }

        // SPIN SHOOTER
        if (gamepad2.dpadUpWasPressed()) {
            shooterSpinning = !shooterSpinning;
        }

        if (gamepad2.yWasPressed()) {
            sorter.left();
        } else if (gamepad2.bWasPressed()) {
            sorter.right();
        } else if (gamepad2.xWasPressed()) {
            sorter.home();
        }

        if (gamepad2.dpadDownWasPressed()) {
            intakeSpinning = !intakeSpinning;
        }

        intake.setPower(intakeSpinning ? 1 : 0);
        shooter.setShooterPower(shooterSpinning ? power : 0);

        // FLICK BALL INTO SHOOTER
        if (gamepad2.right_bumper) {
            sorter.setFinger(FINGER_ACTIVE);
        } else {
            sorter.setFinger(FINGER_HOME);
        }

        // CHANGE POWER MANUALLY BY .05
        if (gamepad1.leftBumperWasPressed()) {
            power -= .05;
        } else if (gamepad1.rightBumperWasPressed()) {
            power += .05;
        }

        // canLift = gamepad1.left_bumper && gamepad2.left_bumper;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        pinpoint.update();

        telemetry.addData("Power", power);
        telemetry.addData("Speed",speed);
        telemetry.addData("X",pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y",pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Encoder X",pinpoint.getEncoderX());
        telemetry.addData("Encoder Y",pinpoint.getEncoderY());
        telemetry.addData("Heading",pinpoint.getHeading(AngleUnit.DEGREES));
        //telemetryAprilTag();
        telemetry.addData("Elapsed Time", runtime.milliseconds() / 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        //visionPortal.close();
        super.stop();
    }
}