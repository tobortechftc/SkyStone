package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.CombinedOrientationSensor;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * FoundationHook spec:
 */
public class FoundationHook extends Logger<FoundationHook> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo hook;

    private final double HOOK_UP = 0.14;
    private final double HOOK_DOWN = 0.9;

    private boolean hookIsDown = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "foundationHook";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public FoundationHook(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (hook != null)
            hookUp();
    }

    public void configure(Configuration configuration, boolean auto) {
        hook = new AdjustableServo(0,1).configureLogging(
                logTag + ":foundationHook", logLevel
        );
        hook.configure(configuration.getHardwareMap(), "foundationHook");
        configuration.register(hook);
        hookUp();
        // configuration.register(this);
    }

    public void hookUp() {
        hook.setPosition(HOOK_UP);
        hookIsDown = false;
    }

    public void hookDown() {
        hook.setPosition(HOOK_DOWN);
        hookIsDown = true;
    }

    public void hookAuto() {
        if (hookIsDown) {
            hookUp();
        } else {
            hookDown();
        }
    }


    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (hook != null) {
            line.addData("Hook", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return hook.getPosition();
                }
            });
        }
    }

}


