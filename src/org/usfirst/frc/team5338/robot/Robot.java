package org.usfirst.frc.team5338.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements PIDOutput
    {
        private static final AHRS IMU;
        static
            {
                AHRS temp = null;
                try
                    {
                        temp = new AHRS(SPI.Port.kMXP, (byte) 120);
                    }
                catch(RuntimeException e)
                    {
                        DriverStation.reportError("Error instantiating navX-MXP:  " + e.getMessage(), true);
                        System.exit(1);
                    }
                IMU = temp;
            }

        private static final CANTalon DRIVEL1 = new CANTalon(1);
        private static final CANTalon DRIVEL2 = new CANTalon(2);
        private static final CANTalon DRIVER1 = new CANTalon(3);
        private static final CANTalon DRIVER2 = new CANTalon(4);

        private static final Joystick XBOX = new Joystick(0);
        private static final DigitalInput HALL_SENSOR = new DigitalInput(0);
        
        //        private static final double kP = 0.00; // This constant is not final, and needs to be updated!
        //        private static final double kI = 0.00; // This constant is not final, and needs to be updated!
        //        private static final double kD = 0.00; // This constant is not final, and needs to be updated!
        //        private static final double kF = 0.00; // This constant is not final, and needs to be updated!
        //        private static final double kToleranceDegrees = 0.0; // This constant is not final, and needs to be updated!

        private static final int turnAngle = 180;

        private static enum State
            {
            INITIALIZING,
            MOVING_FORWARDS,
            TURNING,
            MOVING_BACKWARDS,
            STOPPING
            }

        private static State currentState;

        //private static PIDController turnController;

        private static float heading;
        private static float targetHeading;
        private static float acceleration;
        private static float peakAcceleration;

        private static double speed;
        private static boolean reverse;

        private static int driveTimeCounter;
        private static int targetAngleCounter;
        
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
        @Override
        public void robotInit()
            {
                while(Robot.IMU.isCalibrating())
                    {
                    }
                Robot.IMU.reset();
                Robot.IMU.zeroYaw();

                Robot.currentState = State.INITIALIZING;
                Robot.heading = Robot.IMU.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", Robot.heading);
                Robot.acceleration = 0;
                SmartDashboard.putNumber("Acceleration in Gs:", Robot.acceleration);
                Robot.peakAcceleration = Robot.acceleration;
                SmartDashboard.putNumber("Peak Acceleration in Gs:", Robot.peakAcceleration);
                Robot.targetHeading = 0;
                SmartDashboard.putNumber("Target Heading in Degrees:", Robot.targetHeading);
                SmartDashboard.putBoolean("Hall Effect Sensor:", Robot.HALL_SENSOR.get());
                
                //                turnController = new PIDController(kP, kI, kD, kF, IMU, this);
                //                turnController.setInputRange(0, 360);
                //                turnController.setOutputRange(-1.0, 1.0);
                //                turnController.setAbsoluteTolerance(kToleranceDegrees);
                //                turnController.setContinuous(true);
            }

        /**
         * This autonomous (along with the chooser code above) shows how to
         * select between different autonomous modes using the dashboard. The
         * sendable chooser code works with the Java SmartDashboard. If you
         * prefer the LabVIEW Dashboard, remove all of the chooser code and
         * uncomment the getString line to get the auto name from the text box
         * below the Gyro You can add additional auto modes by adding additional
         * comparisons to the switch structure below with additional strings. If
         * using the SendableChooser make sure to add them to the chooser code
         * above as well.
         */
        @Override
        public void autonomousInit()
            {
                Robot.driveTimeCounter = 0;
                Robot.speed = 0.25;
                Robot.reverse = false;

                Robot.heading = Robot.IMU.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", Robot.heading);
                Robot.targetHeading = Robot.heading - Robot.turnAngle;
                if(Robot.targetHeading < 0)
                    {
                        Robot.targetHeading += 360;
                    }
                SmartDashboard.putNumber("Target Heading in Degrees:", Robot.targetHeading);
                Robot.currentState = State.MOVING_FORWARDS;
            }

        /**
         * This function is called periodically during autonomous
         */
        @Override
        public void autonomousPeriodic()
            {
                Robot.heading = Robot.IMU.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", Robot.heading);

                switch(Robot.currentState)
                    {
                        case MOVING_FORWARDS:
                            if(Robot.driveTimeCounter < 100)
                                {
                                    Robot.Drive(-1, -1);
                                    Robot.driveTimeCounter++;
                                }
                            if(Robot.driveTimeCounter == 100)
                                {
                                    Robot.currentState = State.TURNING;
                                }

                        case TURNING:
                            float offset = Robot.targetHeading - Robot.heading;
                            if(offset > 180)
                                {
                                    offset -= 360;
                                }
                            if(offset < -180)
                                {
                                    offset += 360;
                                }

                            if(Math.abs(offset / 45) < 0.175)
                                {
                                    Robot.speed = 0.175;
                                }
                            else if(Math.abs(offset / 45) > 0.375)
                                {
                                    Robot.speed = 0.375;
                                }
                            else
                                {
                                    Robot.speed = Math.abs(offset / 45);
                                }

                            if(offset > 2)
                                {
                                    Robot.Drive(-1, 1);
                                }
                            else if(offset < -2)
                                {
                                    Robot.Drive(1, -1);
                                }
                            else
                                {
                                    Robot.Drive(0, 0);
                                    Robot.targetAngleCounter++;
                                    if(Robot.targetAngleCounter == 25)
                                        {
                                            Robot.currentState = State.MOVING_BACKWARDS;
                                            Robot.driveTimeCounter = 0;
                                        }
                                }
                            //                            float offset = Robot.targetHeading - Robot.heading;
                            //                            if(offset > kToleranceDegrees)
                            //                                {
                            //                                    turnController.setSetpoint(targetHeading);
                            //                                    turnController.enable();
                            //                                    double currentRotationRate = rotateToAngleRate;
                            //                                    Robot.Drive(currentRotationRate, -1 * currentRotationRate);
                            //                                    Timer.delay(0.005);
                            //                                    targetAngleCounter = 0;
                            //                                }
                            //                            else
                            //                                {
                            //                                    Robot.Drive(0, 0);
                            //                                    Robot.targetAngleCounter++;
                            //                                    if(Robot.targetAngleCounter > 25)
                            //                                        {
                            //                                            Robot.currentState = State.MOVING_BACKWARDS;
                            //                                            Robot.driveTimeCounter = 0;
                            //                                        }
                            //                                }

                        case MOVING_BACKWARDS:
                            if(Robot.driveTimeCounter < 100)
                                {
                                    Robot.Drive(-1, -1);
                                    Robot.driveTimeCounter++;
                                }
                            if(Robot.driveTimeCounter == 100)
                                {
                                    Robot.currentState = State.STOPPING;
                                }

                        default:
                            Robot.Drive(0, 0);
                    }
            }

        /**
         * This function is called periodically during operator control
         */
        @Override
        public void teleopPeriodic()
            {
                SmartDashboard.putBoolean("Hall Effect Sensor:", Robot.HALL_SENSOR.get());

                // LB - full speed
                // RB - half speed

                if(Robot.XBOX.getRawButton(5))
                    {
                        Robot.speed = 1;
                    }
                if(Robot.XBOX.getRawButton(6))
                    {
                        Robot.speed = 0.5;
                    }

                Robot.Drive(Robot.XBOX.getRawAxis(1), Robot.XBOX.getRawAxis(3));

                // BACK - reverse
                // START - forwards

                if(Robot.XBOX.getRawButton(7))
                    {
                        Robot.reverse = true;
                    }
                if(Robot.XBOX.getRawButton(7) == true)
                    {
                        Robot.XBOX.setRumble(Joystick.RumbleType.kLeftRumble, 1);
                    }
                else
                    {
                        Robot.XBOX.setRumble(Joystick.RumbleType.kLeftRumble, 0);
                    }

                if(Robot.XBOX.getRawButton(8))
                    {
                        Robot.reverse = false;
                    }

                Robot.heading = Robot.IMU.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", Robot.heading);
                Robot.acceleration = Robot.IMU.getWorldLinearAccelX();
                SmartDashboard.putNumber("Acceleration in Gs:", Robot.acceleration);
                if(Robot.peakAcceleration < Robot.acceleration)
                    {
                        Robot.peakAcceleration = Robot.acceleration;
                        SmartDashboard.putNumber("Peak Acceleration in Gs:", Robot.peakAcceleration);
                    }
            }

        @Override
        public void pidWrite(double output)
            {
            }

        private static void Drive(double driveL, double driveR)
            {
                if(Robot.reverse)
                    {
                        Robot.DRIVEL1.set(driveR * Robot.speed * -1);
                        Robot.DRIVEL2.set(driveR * Robot.speed * -1);
                        Robot.DRIVER1.set(driveL * Robot.speed);
                        Robot.DRIVER2.set(driveL * Robot.speed);
                    }
                else
                    {
                        Robot.DRIVEL1.set(driveL * Robot.speed * -1);
                        Robot.DRIVEL2.set(driveL * Robot.speed * -1);
                        Robot.DRIVER1.set(driveR * Robot.speed);
                        Robot.DRIVER2.set(driveR * Robot.speed);
                    }
            }
    }
