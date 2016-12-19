package org.usfirst.frc.team5338.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
    {
        AHRS sensor = new AHRS(SPI.Port.kMXP, (byte) 200);
        DigitalInput sensor2 = new DigitalInput(0);

        CANTalon DriveL1 = new CANTalon(1);
        CANTalon DriveL2 = new CANTalon(2);
        CANTalon DriveR1 = new CANTalon(3);
        CANTalon DriveR2 = new CANTalon(4);

        Joystick xbox = new Joystick(0);

        double speed;
        boolean button;
        boolean reverse;
        int autoCounter;
        float heading;
        float targetHeading;
        float accel;
        float accelPeak;
        float targetCounter;
        int stateCount;

        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
        @Override
        public void robotInit()
            {
                while(this.sensor.isCalibrating())
                    {
                    }
                this.sensor.reset();
                this.sensor.zeroYaw();

                this.heading = this.sensor.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", this.heading);
                this.accel = 0;
                SmartDashboard.putNumber("Acceleration in G's:", this.accel);
                this.accelPeak = this.accel;
                SmartDashboard.putNumber("Peak Acceleration in G's:", this.accelPeak);
                this.targetHeading = 0;
                SmartDashboard.putNumber("Target Heading in Degrees:", this.targetHeading);
                SmartDashboard.putBoolean("Hall Effect Sensor:", this.sensor2.get());
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
                this.autoCounter = 0;
                this.targetCounter = 0;
                this.stateCount = 0;
                this.reverse = false;
                this.speed = 0.25;

                this.heading = this.sensor.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", this.heading);
                this.targetHeading = this.heading - 180;
                if(this.targetHeading < 0)
                    {
                        this.targetHeading += 360;
                    }
                SmartDashboard.putNumber("Target Heading in Degrees:", this.targetHeading);
            }

        /**
         * This function is called periodically during autonomous
         */
        @Override
        public void autonomousPeriodic()
            {
                this.heading = this.sensor.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", this.heading);

                if(this.stateCount == 0)
                    {
                        if(this.autoCounter < 100)
                            {
                                this.Drive(-1, -1);
                                this.autoCounter++;
                            }
                        if(this.autoCounter == 200)
                            {
                                this.stateCount++;
                            }
                    }

                if(this.stateCount == 1)
                    {
                        float offset = this.targetHeading - this.heading;
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
                                this.speed = 0.175;
                            }
                        else if(Math.abs(offset / 45) > 0.375)
                            {
                                this.speed = 0.375;
                            }
                        else
                            {
                                this.speed = Math.abs(offset / 45);
                            }

                        if(offset > 2)
                            {
                                this.Drive(-1, 1);
                            }
                        else if(offset < -2)
                            {
                                this.Drive(1, -1);
                            }
                        else
                            {
                                this.Drive(0, 0);
                                this.targetCounter++;
                                if(this.targetCounter > 50)
                                    {
                                        this.stateCount++;
                                        this.autoCounter = 0;
                                    }
                            }
                    }

                if(this.stateCount == 2)
                    {
                        if(this.autoCounter < 100)
                            {
                                this.Drive(-1, -1);
                                this.autoCounter++;
                            }
                        if(this.autoCounter == 200)
                            {
                                this.stateCount++;
                            }
                    }

                if(this.stateCount == 3)
                    {
                        this.Drive(0, 0);
                    }
            }

        /**
         * This function is called periodically during operator control
         */
        @Override
        public void teleopPeriodic()
            {
                SmartDashboard.putBoolean("Hall Effect Sensor:", this.sensor2.get());

                // LB - full speed
                // RB - half speed

                if(this.xbox.getRawButton(5))
                    {
                        this.speed = 1;
                    }
                if(this.xbox.getRawButton(6))
                    {
                        this.speed = 0.5;
                    }

                this.Drive(this.xbox.getRawAxis(1), this.xbox.getRawAxis(3));

                // BACK - reverse
                // START - forwards

                if(this.xbox.getRawButton(7))
                    {
                        this.reverse = true;
                    }
                if(this.xbox.getRawButton(7) == true)
                    {
                        this.xbox.setRumble(Joystick.RumbleType.kLeftRumble, 1);
                    }
                else
                    {
                        this.xbox.setRumble(Joystick.RumbleType.kLeftRumble, 0);
                    }

                if(this.xbox.getRawButton(8))
                    {
                        this.reverse = false;
                    }

                this.heading = this.sensor.getFusedHeading();
                SmartDashboard.putNumber("Heading in Degrees:", this.heading);
                this.accel = this.sensor.getWorldLinearAccelX();
                SmartDashboard.putNumber("Acceleration in G's:", this.accel);
                if(this.accelPeak < this.accel)
                    {
                        this.accelPeak = this.accel;
                        SmartDashboard.putNumber("Peak Acceleration in G's:", this.accelPeak);
                    }
            }

        private void Drive(double driveL, double driveR)
            {
                if(this.reverse)
                    {
                        this.DriveL1.set(driveR * this.speed * -1);
                        this.DriveL2.set(driveR * this.speed * -1);
                        this.DriveR1.set(driveL * this.speed);
                        this.DriveR2.set(driveL * this.speed);
                    }
                else
                    {
                        this.DriveL1.set(driveL * this.speed * -1);
                        this.DriveL2.set(driveL * this.speed * -1);
                        this.DriveR1.set(driveR * this.speed);
                        this.DriveR2.set(driveR * this.speed);
                    }
            }

    }
