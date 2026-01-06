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
X -

*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="TestBotTeleOp", group="A")
public class TestBotTeleOp extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private double speed = 1;
    private final double LOW_SPEED = 0.6;
    private final double HIGH_SPEED = 1;
    private boolean blueTeam = true; // TRUE = BLUE TEAM, FALSE = RED TEAM
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

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

        }

    @Override
    public void start() {
        runtime.reset();
    }

    public double getAprilTagPosX() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                return result.getTx();
            }
        }
        return 0;
    }

    @Override
    public void loop() {
        double max;

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        if (gamepad1.a) {
            yaw = Math.min(getAprilTagPosX()/31,1);
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

        if (gamepad1.dpadRightWasPressed()) {
            speed = HIGH_SPEED;
        } else if (gamepad1.dpadLeftWasPressed()) {
            speed = LOW_SPEED;
        }

        if (gamepad1.leftBumperWasPressed()) {
            pinpoint.resetPosAndIMU();
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        pinpoint.update();

        telemetry.addData("Speed",speed);
        telemetry.addData("Inch X",pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Inch Y",pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Encoder X",pinpoint.getEncoderX());
        telemetry.addData("Encoder Y",pinpoint.getEncoderY());
        telemetry.addData("Heading",pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Elapsed Time", runtime.milliseconds() / 1000);
        telemetry.update();
    }
}