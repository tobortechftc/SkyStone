package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.OpModeTerminationException;
import org.firstinspires.ftc.teamcode.support.YieldHandler;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.List;

/**
 * Created by 28761 on 6/29/2019.
 */
@Disabled
@Autonomous(name = "Sigma-Blue-Stone Only", group = "Sigma")
public class SigmaAutoBlueStoneOnly extends LinearOpMode {
    private ToboSigma.SkystoneLocation StoneLoc;

    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public double auto_chassis_power = .4;

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboSigma Autonomous runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboSigma robot = new ToboSigma();
        robot.configureLogging("ToboSigma", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboSigma Autonomous finished log configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, ToboSigma.AutoTeamColor.AUTO_BLUE);
            configuration.apply();
            robot.reset(true);
            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboSigma Autonomous finished initialization (CPU_time = %.2f sec)", getRuntime());
        // Wait for the game to start (driver presses PLAY)
        List<Recognition> updatedRecognitions = null;
//        if (robot.cameraStoneDetector.getTfod()!=null) {
//            updatedRecognitions = robot.cameraStoneDetector.getTfod().getUpdatedRecognitions();
//        }
        int robot_pos = 1;
        if (robot.intake != null)
            robot.intake.intakeDropInit();
        waitForStart();
        robot.runtime.reset();
        // run until the end of the match (driver presses STOP or timeout)
        if (!opModeIsActive()) {
            return;
        }
        try {
            boolean isBlue = true;
            boolean isLeft = false;
            // put autonomous steps here
            // step-1: detect skystone location
            StoneLoc = robot.cameraStoneDetector.getSkystonePositionTF(false);
//            sysout(StoneLoc.toString(), 2000);
            // step-2: go to grab the first skystone and deliver
            robot.approachStone(StoneLoc, isBlue, isLeft);
//            sysout(String.format("first stone X: %f", nextStoneX(StoneLoc, 0)), 2000);
            double dumpping = 120;
            robot.oneMoreStone(true, 0.0, nextStoneX(StoneLoc, 0), dumpping, getStoneId(StoneLoc, 0));

            // step-3: repetitively grab and deliver the next skystone/stone

            int count = 1;
            while (getRuntime() < 25000) {
//                sysout(String.format("%dth stone X: %f", count,nextStoneX(StoneLoc, count)), 2000);
                robot.oneMoreStone(false, dumpping + (count-1) * 10, nextStoneX(StoneLoc, count), dumpping + (count * 10), getStoneId(StoneLoc, count));
                count++;
            }

            // park
            robot.chassis.driveStraightAutoRunToPosition(0.4, 20, +90, 1000);


        } catch (Exception E) {
            telemetry.addData("Error in event handler", E.getMessage());
            handleException(E);
            Thread.sleep(5000);
        }

    }

    static final int[] CENTER_ORDER = {4, 1, 5, 3, 2, 0};
    static final int[] LEFT_ORDER = {5, 2, 4, 3, 1, 0};
    static final int[] RIGHT_ORDER = {3, 5, 4, 2, 1, 0};

    int getStoneId(ToboSigma.SkystoneLocation ssloc, int count) {
        int stoneID;
        if (ssloc == ToboSigma.SkystoneLocation.LEFT)
            stoneID = LEFT_ORDER[count];
        else if (ssloc == ToboSigma.SkystoneLocation.RIGHT)
            stoneID = RIGHT_ORDER[count];
        else
            stoneID = CENTER_ORDER[count];
        return stoneID;
    }

    double nextStoneX(ToboSigma.SkystoneLocation ssloc, int count) {
        int stoneID;
        if (ssloc == ToboSigma.SkystoneLocation.LEFT)
            stoneID = LEFT_ORDER[count];
        else if (ssloc == ToboSigma.SkystoneLocation.RIGHT)
            stoneID = RIGHT_ORDER[count];
        else
            stoneID = CENTER_ORDER[count];
        return (stoneID - 3) * 20.32 - 8.0;
    }

    void sysout(String str, long time) {
        telemetry.addLine(str);
        telemetry.update();
        sleep(time);
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for (StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
