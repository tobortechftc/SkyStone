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
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * StoneGrabber spec:
 */
public class StoneGrabber extends Logger<StoneGrabber> implements Configurable {

    final private CoreSystem core;

    private DcMotor lifter;
    private AdjustableServo arm;
    private AdjustableServo wrist;
    private AdjustableServo grabber;
    private ElapsedTime SGTimer;
    private double waitSec;


    private final double ARM_UP = 0.1;
    private final double ARM_DOWN = 0.9;
    private final double ARM_DOWN_SAFE = 0.9;
    private final double ARM_INITIAL = 0.84;
    private final double ARM_IN = 0.67;
    private final double ARM_LOW = 0.6;
    private final double ARM_OUT = 0.45;
    private final double ARM_DELIVER = 0.3;
    private final double ARM_INC_UNIT = 0.02;

    private final double WRIST_PARALLEL = 0.06; // 0.18;
    private final double WRIST_PERPENDICULAR = 0.62;
    private final double WRIST_INIT = WRIST_PERPENDICULAR;

    private final double GRABBER_INIT = 0.31;
    private final double GRABBER_OPEN_IN = 0.6;
    private final double GRABBER_OPEN = 0.92;
    private final double GRABBER_CLOSE = 0.31;

    //private final int LIFT_RUN_TO_POSITION_OFFSET = 20;   // V5.2 
    private final int LIFT_RUN_TO_POSITION_OFFSET = 100;  // V5.3, new control for goBilda 5205 motor
    private final int LIFT_DOWN = 20;
    private final int LIFT_GRAB = 400;
    private final int LIFT_GRAB_AUTO = 640;
    private final int LIFT_MAX = 3640;
    private final int LIFT_SAFE_SWING_AUTO = 850;
    private final int LIFT_SAFE_BRIDGE = 800;
    private final int LIFT_SAFE_SWING_IN = LIFT_SAFE_SWING_AUTO+400;
    private final int LIFT_SAFE_SWING = LIFT_SAFE_SWING_AUTO;
    //private final double LIFT_POWER = 0.5;   // V5.2
    private final double LIFT_POWER = 1.0;  // V5.3
    private final double LIFT_POWER_SLOW = 0.5;
    private final int LIFT_DELIVER = 1000;


    private boolean armIsDown = false;
    private boolean armIsIn = true;
    private boolean isGrabberOpened = false;
    private boolean isWristParallel = false;
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
            armInit();
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
        armInit();

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
        //if (lifter != null) lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        if (lifter != null) lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // register hanging as configurable component
        // configuration.register(this);
    }

    public boolean isArmInside() {
        return armIsIn;
    }

    public boolean isArmDown() {
        return armIsDown;
    }

    public void armInit() {
        arm.setPosition(ARM_INITIAL);
        armIsDown = false;
        armIsIn = true;
    }

    public void armOut() {
        arm.setPosition(ARM_OUT);
        armIsDown = false;
        armIsIn = false;
    }

    public void armDownInc() {
        double cur_pos = arm.getPosition();
        cur_pos += ARM_INC_UNIT;
        if (cur_pos>1) cur_pos=1;
        arm.setPosition(cur_pos);
        armIsDown = false;
        if (cur_pos>ARM_IN)
            armIsIn = true;
        else
            armIsIn = false;
        if (Math.abs(cur_pos-ARM_DOWN)<0.2)
            armIsDown = true;
    }

    public void armUpInc() {
        double cur_pos = arm.getPosition();
        cur_pos -= ARM_INC_UNIT;
        if (cur_pos<0) cur_pos=0;
        arm.setPosition(cur_pos);
        armIsDown = false;
        if (cur_pos<ARM_LOW)
            armIsIn = false;
        else
            armIsIn = true;
        if (Math.abs(cur_pos-ARM_DOWN)<0.2)
            armIsDown = true;
    }

    public void armUp() {
        arm.setPosition(ARM_UP);
        armIsDown = false;
        armIsIn = false;
    }

    public void armDown() {
        arm.setPosition(ARM_DOWN);
        armIsDown = true;
        armIsIn = true;
    }

    public void armDeliver() {
        arm.setPosition(ARM_DELIVER);
        armIsDown = false;
        armIsIn = false;
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
        isWristParallel = false;
    }

    public void wristParallel () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PARALLEL);
        isWristParallel = true;
    }

    public void wristPerpendicular () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PERPENDICULAR);
        isWristParallel = false;
    }

    public void wristAuto() {
        if (isWristParallel) wristPerpendicular();
        else wristParallel();
    }

    public Progress moveWrist(boolean parallel) {
        double target = parallel ? WRIST_PARALLEL : WRIST_PERPENDICULAR;
        isWristParallel = parallel;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveWrist(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from parallel to vertical takes 2 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(800 * adjustment);
        wrist.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void grabberInit() {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_INIT);
    }

    public void grabberOpen () {
        if (grabber==null) return;
        if (armIsIn)
            grabber.setPosition(GRABBER_OPEN_IN);
        else
            grabber.setPosition(GRABBER_OPEN);
        isGrabberOpened = true;
    }

    public void grabberClose () {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_CLOSE);
        isGrabberOpened = false;
    }

    public void grabberAuto() {
        if (isGrabberOpened)
            grabberClose();
        else
            grabberOpen();
    }

    public Progress moveGrabber(boolean closed) {
        double target = GRABBER_CLOSE;
        if (!closed) {
           if (armIsIn)
               target = GRABBER_OPEN_IN;
           else
               target = GRABBER_OPEN;
        }
        isGrabberOpened = !closed;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveGrabber(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from up to down takes 1 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(800 * adjustment);
        grabber.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void liftToPosition(int pos, double power) {
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(pos);
        lifter.setPower(power);
    }

    public void liftUp (boolean slow) {
        if (lifter==null) return;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (slow)
            lifter.setPower(LIFT_POWER_SLOW);
        else
            lifter.setPower(LIFT_POWER);
    }

    public void liftDown(boolean slow) {
        if (lifter==null) return;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (slow)
            lifter.setPower(-LIFT_POWER_SLOW);
        else
            lifter.setPower(-LIFT_POWER);
    }

    public void liftStop() {
        if (lifter==null) return;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = lifter.getCurrentPosition();
        liftToPosition(pos);
        // lifter.setPower(0);
    }

    public void liftToSafe() {
        liftToPosition(LIFT_SAFE_SWING);
    }

    public Progress liftToPosition(int pos) {
        lifter.setPower(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(pos);
        lifter.setPower(LIFT_POWER);
        return new Progress() {
            public boolean isDone() {
                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET; 
            }
        };
    }

    private Progress moveArm(double position) {
        double adjustment = Math.abs(position - arm.getPosition());
        arm.setPosition(position);
        if (position>=ARM_IN)
            armIsIn=true;
        else
            armIsIn=false;
        if (Math.abs(position-ARM_DOWN)<0.2)
            armIsDown = true;
        else
            armIsDown = false;
        // 3.3ms per degree of rotation
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 900);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void armOutComboAuto() {
        if (Thread.currentThread().isInterrupted()) return;
        armOutCombo();
        while (!TaskManager.isComplete("Arm Out Combo")) {
            TaskManager.processTasks();
        }
    }

    public void waitTM(double sec) {
        final String taskName = "Wait";
        if (!TaskManager.isComplete(taskName)) return;
        waitSec = sec;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                SGTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                };
            }
        }, taskName);
    }
    public void armOutCombo(){
        armOutCombo(0);
    }
    public void armOutCombo(double delaySec) {
        final String taskName = "Arm Out Combo";
        if (!TaskManager.isComplete(taskName)) return;
        boolean grabIsOpened = isGrabberOpened;
        if (delaySec>0) {
            waitSec = delaySec;
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    SGTimer.reset();
                    return new Progress() {
                        @Override
                        public boolean isDone() { return (SGTimer.seconds() >= waitSec); }
                    };
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                if (isWristParallel && isGrabberOpened) grabberClose();
                if (arm.getPosition()>ARM_LOW) // arm inside the robot
                    liftToPosition(LIFT_SAFE_SWING);
                else
                    liftToPosition(LIFT_GRAB);

                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_OUT);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_GRAB);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);
        if (grabIsOpened) { // restore grabber to open
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(false);
                }
            }, taskName);
        }
    }
    public void armInComboAuto(final boolean wristParallel) {
        if (Thread.currentThread().isInterrupted()) return;
        armInCombo(wristParallel, true);
        while (!TaskManager.isComplete("Arm In Combo")) {
            TaskManager.processTasks();
        }
    }
    public void armInCombo(final boolean wristParallel, boolean isAuto) {
        final String taskName = "Arm In Combo";
        if (!TaskManager.isComplete(taskName)) return;
        boolean grabIsOpened = isGrabberOpened;
        if (wristParallel!=isWristParallel) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    moveWrist(wristParallel);
                    grabberClose();
                    return new Progress() {
                        @Override
                        public boolean isDone() {
                            return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                        }
                    };
                }
            }, taskName);
        }
        if (lifter.getCurrentPosition()<LIFT_SAFE_SWING) {
            if (isAuto) {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        liftToPosition(LIFT_SAFE_SWING_AUTO);
                        return new Progress() {
                            @Override
                            public boolean isDone() {
                                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                            }
                        };
                    }
                }, taskName);
            } else {
                TaskManager.add(new Task() {
                    @Override
                    public Progress start() {
                        liftToPosition(LIFT_SAFE_SWING);
                        return new Progress() {
                            @Override
                            public boolean isDone() {
                                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                            }
                        };
                    }
                }, taskName);
            }
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN_SAFE);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                liftToPosition(LIFT_DOWN);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
                    }
                };

            }
        }, taskName);
        if (wristParallel && grabIsOpened) { // restore grabber to open
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveGrabber(false);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN);
            }
        }, taskName);


    }

    public void grabStoneComboAuto() {
        if (Thread.currentThread().isInterrupted()) return;
        grabStoneCombo();
        while (!TaskManager.isComplete("Grab Stone Combo")) {
            TaskManager.processTasks();
        }
    }
    public void grabStoneCombo() {
        final String taskName = "Grab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_DOWN);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 50;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_OUT);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress grabberProgress = moveGrabber(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return grabberProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                arm.setPosition(ARM_DELIVER);
                liftToPosition(LIFT_DOWN);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);

    }

    public void grabStoneInsideCombo() {
        final String taskName = "Grab Stone Inside Combo";
        if (!TaskManager.isComplete(taskName)) return;

        armInReadyGrabCombo();

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_DOWN);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < LIFT_RUN_TO_POSITION_OFFSET;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return true;
                    }
                };
            }
        }, taskName);

    }

    public void armInReadyGrabCombo() {
        final String taskName = "Arm In Ready Grab Combo";
        if (!TaskManager.isComplete(taskName)) return;
        final boolean armWasIn = armIsIn;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                int position = LIFT_SAFE_SWING_IN;
                if (armWasIn)
                    position = LIFT_SAFE_BRIDGE;
                liftToPosition(position);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 50;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                final Progress wristProgress = moveWrist(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return wristProgress.isDone();}
                };
            }
        }, taskName);
        if (!armWasIn) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    int position = LIFT_SAFE_BRIDGE;
                    liftToPosition(position);
                    return new Progress() {
                        @Override
                        public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 50;
                        }
                    };
                }
            }, taskName);
        }
    }

    public void deliverStoneComboAuto() {
        if (Thread.currentThread().isInterrupted()) return;
        deliverStoneCombo(true);
        while (!TaskManager.isComplete("Deliver Stone Combo")) {
            TaskManager.processTasks();
        }
    }

    public void deliverStoneCombo(boolean auto) {
        final String taskName = "Deliver Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (auto) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_DELIVER);
                }
            }, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return moveArm(ARM_OUT);
                }
            }, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen(); return liftToPosition(LIFT_SAFE_SWING);
            }
        }, taskName);
    }
    /*
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
            line.addData("/armIn", "=%s", new Func<String>() {
                @Override
                public String value() {
                    return ((armIsIn?"T":"F"));
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
        if (lifter != null) {
            line.addData("lifter", "pos=%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return lifter.getCurrentPosition();
                }
            });
        }

    }

}



