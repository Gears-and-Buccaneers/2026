# Robot Code Overview

A high-level overview of the robot code called (during teleop).

```txt
Scurvy.teleopPeriodic()                                        | Just during teleop mode: use the controllers
  Scurvy.manuallyDrive()                                       |
    if DriverController.shouldBrake()                          | Does the driver want to brake?
      Drivetrain.brake()                                       | Record desired braking
    …if not braking…                                           |
      Drivetrain.drive()                                       | Record desired field-centric driving
    if DriverController.shouldZeroGyro()                       | Does the driver want to zero the robot's gyro heading?
      Drivetrain.zeroHeading()                                 | I swear the bot is exactly facing the other alliance now
  Scurvy.manuallyOperate()                                     |
    if OperatorController.shouldToggleLEDMode()                | Does the operator want to toggle manual lighting?
      …toggle an internal variable                             |
    if OperatorController.shouldSetFallbackShooterSpinSpeed()  | Does the operator want to spin the shooter at a fixed speed?
      Shooter.fallbackSpin()                                   | Set the shooter motor spin rate to a target value
    elif OperatorController.shouldSmartAim()                   | Does the operator want to use smart aim mode?
      Scurvy.dynamicallyTargetHub()                            |
        Drivetrain.getVelocity()                               | How fast are we moving?
        Shooter.calculateShootingSolution()                    | Where should we point and how fast should we launch to score?
        Shooter.setTargetMuzzleSpeed()                         | Make the shooter spin at the right speed
        Drivetrain.driveFacingAngle()                          | Strafe while rotating to face a target angle
    …if not smart aiming…                                      |
        Shooter.spinDown()                                     | Let the shooter flywheel stop spinning

Scurvy.robotPeriodic()                                         | Always: update vision and LEDs
  …if this is during simulation…                               |
    Drivetrain.getPose()                                       | Where does the drivetrain say we are?
    Vision.update_sim()                                        | Update vision sim with the pose
  Vision.get_measurements()                                    | What is the vision system seeing?
  Drivetrain.addVisionMeasurement()                            | Update where we think we are using the vision, too
  Scurvy.maybeSetOperatorPerspective()                         | Check if the robot knows where it's facing
    …if the robot doesn't know which alliance it is on…        |
      Drivetrain.setOperatorPerspectiveForwardOrientation()    | Tell the bot which way is forward
  Scurvy.updateLights()                                        |
    …if the operator asked for manual lighting…                |
      operatorController.customLEDColor()                      | Ask the controller what color to show
      Lighting.setColor()                                      | Set the LED color to what the controller said
    …otherwise…                                                |
      Scurvy.currentShift()                                    | What shift is it (for teleop)?
      Scurvy.hubIsActive()                                     | Can we score in the hub right now?
      Lighting.showShift()                                     | Show colored progress using time, phase, shift, and hub

Components' execute() is called after the periodic methods:
  Drivetrain.execute()                                         | Apply the current drive request to the drive motors
  Shooter.execute()                                            | Get the shooter motor(s) to the requested speed(s)
  Intake.execute()                                             | Get the intake motor(s) to the requested speed(s)
  Lighting.execute()                                           | Make the LED hardware show requested color and progress
  Vision.execute()                                             | Convert what each camera is seeing to a VisionMeasurement
```
