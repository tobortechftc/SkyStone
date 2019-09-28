package org.firstinspires.ftc.teamcode.opmodes.sigmaBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * Created by 28761 on 6/29/2019.
 */
@Disabled
@Autonomous(name="Sigma-Auto", group="Sigma")
public class SigmaAutoTemplate extends LinearOpMode {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public SwerveChassis chassis;
    public double auto_chassis_power = .6;

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
            robot.configure(configuration, telemetry, true);
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
        waitForStart();
        robot.runtime.reset();
        // run until the end of the match (driver presses STOP or timeout)
        while (opModeIsActive()) {
            try {
                // put autonomous steps here
                // step-1: detect skystone location
                // step-2: go to gran the first skystone
                // step-3: deliver the first skystone
                //
                int skyStonePosition = 3; // Collin, this is for your function
                getFirstSkyStone(skyStonePosition);
                int count = 1;
                while (getRuntime() < 25000){
                    count++;
                    getAnotherSkyStone(skyStonePosition, count);
                }
                // move foundation
                // park

            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
    }
    public void getFirstSkyStone(int skyStonePosition) throws InterruptedException{
        chassis.driveStraightAuto(auto_chassis_power, 65, 0, 10000);
        if(skyStonePosition == 1){
            chassis.driveStraightAuto(auto_chassis_power, 1, 90, 10000);  // test to get exact numbers
        } else if(skyStonePosition == 2){
            chassis.driveStraightAuto(auto_chassis_power, 1, 90, 10000);  // test to get exact numbers
        } else {
            chassis.driveStraightAuto(auto_chassis_power, 1, 90, 10000);  // test to get exact numbers
        }
        //grab skystone
        chassis.driveStraightAuto(auto_chassis_power, 220 + 20 *skyStonePosition, -90, 15000);//probably too much
        //place skystone
    }
    public void getAnotherSkyStone(int skyStonePosition, int stoneNum) throws InterruptedException{//stoneNum - how many stones ara we going to have after this trip
        int toTake;
        if (skyStonePosition == 1) {
           int [] a = {4, 2, 3, 5, 6};
           toTake = a[stoneNum - 2];
        } else if (skyStonePosition == 2) {
            int [] a = {5, 1, 3, 4, 6};
            toTake = a[stoneNum - 2];
        } else {
            int[] a = {6, 1, 2, 4, 5};
            toTake = a[stoneNum - 2];
        }
        chassis.driveStraightAuto(auto_chassis_power, 260 + 20 * stoneNum, 90, 15000);//numbers - probably not correct
        chassis.driveStraightAuto(auto_chassis_power, 20, 0, 10000);
        chassis.driveStraightAuto(auto_chassis_power, -5, 0, 10000);
        //grab stone
        chassis.driveStraightAuto(auto_chassis_power, 243 + 20 * stoneNum, -90, 15000);
        // place stone on foundation

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
