package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboTest;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.OpModeTerminationException;
import org.firstinspires.ftc.teamcode.support.YieldHandler;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * Created by 28761 on 6/29/2019.
 */
@Disabled
@TeleOp(name="TFOD-TeleOp", group="Test")
public class TFOD_TeleOp extends LinearOpMode implements YieldHandler {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    private EventManager eventManager1;


    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboSigma TeleOp runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboSigma.SkystoneLocation StoneLoc= ToboSigma.SkystoneLocation.UNKNOWN;

        ToboSigma robot = new ToboSigma();
        robot.configureLogging("ToboSigma",LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
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
        log.info("RoboSigma TeleOp finished initialization (CPU_time = %.2f sec)", getRuntime());
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.core.set_yield_handler(this); // uses this class as yield handler
        int count=0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double ini_time = getRuntime();
            count++;
            StoneLoc = robot.cameraStoneDetector.getSkystonePositionTF(false);
            telemetry.addData("StoneLoc", "%s, time=%.2f, count=%1d",
                    StoneLoc.toString(),(getRuntime()-ini_time), count);
            telemetry.update();
            sleep(100);
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

    public void on_yield() {
        if (!opModeIsActive())
            throw new OpModeTerminationException();
    }
}
