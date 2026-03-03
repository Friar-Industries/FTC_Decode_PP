/*
PRELOAD GREEN IN BOTTOM, PURPS ON TOP
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

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

@Autonomous(name = "BlueBetterAuton", group = "A")
public class BlueBetterAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Integer pathState;
    private double preCollect = 52;
    private double[] artifactY = {84};
    private double[] artifactX = {39,34,29};
    private double artifactHeading = 0;
    private final Pose startPose = new Pose(26,132,Math.toRadians(54));
    private final Pose obeliskPose = new Pose(49,116,Math.toRadians(54));
    private final Pose shootPose = new Pose(59.35,87.28,Math.toRadians(127));
    private final Pose collectPose = new Pose(preCollect,artifactY[0],Math.toRadians(artifactHeading));
    private final Pose preCollectPose = new Pose(42,artifactY[0],Math.toRadians(artifactHeading));
    private final Pose artifact1Pose = new Pose(artifactX[0],artifactY[0], Math.toRadians(artifactHeading));
    private final Pose artifact2Pose = new Pose(artifactX[1], artifactY[0],Math.toRadians(artifactHeading));
    private final Pose artifact3Pose = new Pose(artifactX[2], artifactY[0],Math.toRadians(artifactHeading));
    private final Pose endPose = new Pose(48,73,Math.toRadians(125));
    private Path firstPath;
    private Path secondPath;
    private Path thirdPath;
    private Path extraPath;
    private Path fourthPath;
    private Path fifthPath;
    private Path sixthPath;
    private Path seventhPath;
    private Path eighthPath;
    private DcMotor intake;
    private Shooter shooter = new Shooter();
    private Sorter sorter = new Sorter();
    private Limelight3A limelight;
    private int[] pattern = {0,0,0};
    private String patternState = "PPG";
    private int sorterTimeout = 1000;

    public void buildPaths() {
        firstPath = new Path(new BezierLine(startPose,obeliskPose));
        firstPath.setLinearHeadingInterpolation(startPose.getHeading(),obeliskPose.getHeading());
        secondPath = new Path(new BezierLine(obeliskPose,shootPose));
        secondPath.setLinearHeadingInterpolation(obeliskPose.getHeading(),shootPose.getHeading());
        thirdPath = new Path(new BezierLine(shootPose,collectPose));
        thirdPath.setLinearHeadingInterpolation(shootPose.getHeading(),collectPose.getHeading());
        extraPath = new Path(new BezierLine(collectPose,preCollectPose));
        extraPath.setLinearHeadingInterpolation(collectPose.getHeading(),preCollectPose.getHeading());
        fourthPath = new Path(new BezierLine(preCollectPose,artifact1Pose));
        fourthPath.setLinearHeadingInterpolation(preCollectPose.getHeading(),artifact1Pose.getHeading());
        fifthPath = new Path(new BezierLine(artifact1Pose,artifact2Pose));
        fifthPath.setLinearHeadingInterpolation(artifact1Pose.getHeading(),artifact2Pose.getHeading());
        sixthPath = new Path(new BezierLine(artifact2Pose,artifact3Pose));
        sixthPath.setLinearHeadingInterpolation(artifact2Pose.getHeading(),artifact3Pose.getHeading());
        seventhPath = new Path(new BezierLine(artifact3Pose,shootPose));
        seventhPath.setLinearHeadingInterpolation(artifact3Pose.getHeading(),shootPose.getHeading());
        eighthPath = new Path(new BezierLine(shootPose,endPose));
        eighthPath.setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading());

    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(firstPath);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    pattern = readObelisk();

                    if (pattern[0] == 1) {
                        patternState = "GPP";
                    } else if (pattern[1] == 1) {
                        patternState = "PGP";
                    }

                    follower.followPath(secondPath,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    intake.setPower(0.75);
                    shooter.setShooterPower(0.7);
                    sleep(750);
                    sorter.shootBallsInPattern(patternState);
                    shooter.setShooterPower(0);
                    sorter.left();
                    intake.setPower(0.75);

                    follower.setMaxPower(0.75);
                    follower.followPath(thirdPath,true);
                    setPathState(8);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.setMaxPower(.5);

                    follower.followPath(fourthPath,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    sleep(sorterTimeout);
                    sorter.right();
                    sleep(sorterTimeout);

                    follower.followPath(fifthPath,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    sleep(sorterTimeout);
                    sorter.home();
                    sleep(sorterTimeout);

                    follower.followPath(sixthPath,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    sleep(sorterTimeout+500);
                    intake.setPower(0);

                    follower.setMaxPower(1);
                    follower.followPath(seventhPath,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    intake.setPower(0.75);
                    shooter.setShooterPower(0.7);
                    sorter.shootBallsInPattern(patternState);
                    shooter.setShooterPower(0);
                    intake.setPower(0);

                    follower.followPath(eighthPath,true);
                    setPathState(9);
                }
                break;
            case 8:
                if(!follower.isBusy()) {

                    follower.followPath(extraPath,true);
                    setPathState(3);
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
