package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.util.List;

/**
 * Created by 28761 on 6/29/2019.
 */

@Autonomous(name = "Auto Backup", group = "Sigma")
public class AutoBackup extends LinearOpMode {
    private ToboSigma.SkystoneLocation StoneLoc;
    private EventManager em1;

    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public double auto_chassis_power = .4;

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboSigma Autonomous runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        em1 = new EventManager(gamepad1, true);
        ToboSigma robot = new ToboSigma();
        robot.AutoBackup(em1);

        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        robot.configureLogging("ToboSigma", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboSigma Autonomous finished log configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, ToboSigma.AutoTeamColor.AUTO_BLUE);
            configuration.apply();
            robot.reset(true);
            telemetry.addData("Robot is ready to configure", "Press <A/B/Y/X>");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }

        while (!robot.autoPara.isDone()) { // push X will exit the loop
            try {
                em1.processEvents();
                TaskManager.processTasks();
            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
        log.info("RoboSigma Autonomous finished initialization (CPU_time = %.2f sec)", getRuntime());
        // Wait for the game to start (driver presses PLAY)
        List<Recognition> updatedRecognitions = null;
//        if (robot.cameraStoneDetector.getTfod()!=null) {
//            updatedRecognitions = robot.cameraStoneDetector.getTfod().getUpdatedRecognitions();
//        }
        int robot_pos = 1;
        telemetry.addData("Robot is ready", "Press Play");
        telemetry.update();

        waitForStart();
        robot.runtime.reset();
        // run until the end of the match (driver presses STOP or timeout)
        if (!opModeIsActive()) {
            return;
        }
        try {
            robot.autoFoundationOnly(robot.autoPara.isBlue(), robot.autoPara.isLaneFront(), robot.autoPara.isOffensive());
        } catch (Exception E) {
            telemetry.addData("Error in event handler", E.getMessage());
            handleException(E);
            Thread.sleep(5000);
        }

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
