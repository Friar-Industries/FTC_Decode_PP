/*
PRELOAD GREEN IN BOTTOM, PURPS ON TOP
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Sorter;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "RedFarAuton", group = "A")
public class RedFarAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Integer pathState;
    private final Pose startPose = new Pose(87.5,8.5,Math.toRadians(90));
    private final Pose shootPose = new Pose(87.5,14,Math.toRadians(64));
    private final Pose parkPose = new Pose(87.5,30,Math.toRadians(64));
    private Path firstPath;
    private Path secondPath;
    private DcMotor intake;
    private Shooter shooter = new Shooter();
    private Sorter sorter = new Sorter();
    private Limelight3A limelight;
    private int[] pattern = {0,0,0};
    private String patternState = "PPG";

    public void buildPaths() {
        firstPath = new Path(new BezierLine(startPose,shootPose));
        firstPath.setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading());
        secondPath = new Path(new BezierLine(shootPose,parkPose));
        secondPath.setLinearHeadingInterpolation(shootPose.getHeading(),parkPose.getHeading());
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(firstPath);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    intake.setPower(0.75);
                    shooter.setShooterPower(0.82);
                    sorter.shootBallsInPattern(patternState);
                    sorter.shootBallsInPattern(patternState);
                    shooter.setShooterPower(0);
                    intake.setPower(0);

                    follower.followPath(secondPath,true);
                    setPathState(2);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y",follower.getPose().getY());
        telemetry.addData("Heading",follower.getPose().getHeading());
        telemetry.addData("Pattern", Arrays.toString(pattern));
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooter.init(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sorter.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();

        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }
    public int[] readObelisk() {
        pattern = new int[]{0, 0, 0};
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                pattern[id-21] = 1;
            }
        }
        return pattern;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
