package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.CombinedOrientationSensor;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * StoneGrabber spec:
 */
public class StoneGrabber extends Logger<StoneGrabber> implements Configurable {

    final private CoreSystem core;

    private DcMotor lifter;
    private AdjustableServo arm;
    private AdjustableServo wrist;
    private AdjustableServo grabber;

    private final double ARM_UP = 0.9;
    private final double ARM_DOWN = 0.1;

    private final double WRIST_INIT = 0.5;

    private final double GRABBER_INIT = 0.5;

    private boolean armIsDown = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "StoneGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public StoneGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (arm != null)
            armUp();
        if (wrist!=null)
            wristInit();
        if (grabber!=null)
            grabberInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        arm = new AdjustableServo(0,1).configureLogging(
                logTag + ":arm", logLevel
        );
        arm.configure(configuration.getHardwareMap(), "arm");
        configuration.register(arm);
        armUp();

        wrist = new AdjustableServo(0,1).configureLogging(
                logTag + ":wrist", logLevel
        );
        wrist.configure(configuration.getHardwareMap(), "wrist");
        configuration.register(wrist);
        wristInit();

        grabber = new AdjustableServo(0,1).configureLogging(
                logTag + ":grabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);
        grabberInit();

        lifter = configuration.getHardwareMap().tryGet(DcMotor.class, "lifter");
        if (lifter != null) lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        // register hanging as configurable component
        // configuration.register(this);
    }

    public void armUp() {
        arm.setPosition(ARM_UP);
        armIsDown = false;
    }

    public void armDown() {
        arm.setPosition(ARM_DOWN);
        armIsDown = true;
    }

    public void armAuto() {
        if (armIsDown) {
            armUp();
        } else {
            armDown();
        }
    }

    public void wristInit() {
        if (wrist==null) return;
        wrist.setPosition(WRIST_INIT);
    }

    public void grabberInit() {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_INIT);
    }
    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (arm != null) {
            line.addData("arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return arm.getPosition();
                }
            });
        }
        if (wrist != null) {
            line.addData("wrist", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return wrist.getPosition();
                }
            });
        }
        if (grabber != null) {
            line.addData("grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }
    }

}



