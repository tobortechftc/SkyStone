package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.util.List;

/**
 * Created by 28761 on 6/29/2019.
 */

@Autonomous(name="Sigma-Red-Left", group="Sigma")
public class SigmaAutoRedLeft extends LinearOpMode {
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
        robot.configureLogging("ToboSigma",LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboSigma Autonomous finished log configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, ToboSigma.AutoTeamColor.AUTO_RED);
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

        waitForStart();
        robot.runtime.reset();
        // run until the end of the match (driver presses STOP or timeout)
        if (opModeIsActive()) {
            try {
                boolean isBlue = false;
                boolean isLeft = false;
                // put autonomous steps here
                // step-1: detect skystone location
                StoneLoc = robot.chassis.skyStoneLocation(isBlue);
                // robot.cameraStoneDetector.getSkystonePositionTF(robot_pos);
                // telemetry.addLine(StoneLoc.toString());
                // telemetry.update();
                // sleep(10000); // 10 sec

                // step-2: go to grab the first skystone and deliver
                robot.getFirstSkyStone(StoneLoc, isBlue, isLeft);
                int count = 1;

                // step-3: grab and deliver the next skystone/stone
               /* if (getRuntime() < 25000){
                    count++;
                    robot.getAnotherSkyStone(StoneLoc, count, isBlue);
                }*/
               robot.grabAndPark(isBlue);
                // move foundation
                // park

            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for(StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
