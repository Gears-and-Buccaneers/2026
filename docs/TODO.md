TODO:
[x] Write logs from RIO to USB stick
[x] PID loop on shooter motors to target a RPM
[ ] Connect PDH (and LEDs) to the RoboRIO can bus   -> RoboRIO -> CANdle -> PDH; switch the ... to make it self-terminating
    * So that we can get logs from the PDH for which motors are browning out.
[x] Lower the logging rate of the motors to not overwhelm the RoboRIO with messages
    * So that we have a full log of the match instead of just the last N seconds.
[ ] Lower the current limits on the drivetrain motors
    * So that we don't brown out JUST DRIVING