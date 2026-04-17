# Theseus CAN Devices

The order in which the devices are connected to the CAN FD bus, and their CAN IDs:

Swerve naming below uses current logical corners with shooter edge as robot-front.

| order | device                        | ID |
| ----- | ----------------------------- | -- |
|   a   | Pigeon 2.0                    | 19 |
|   b   | Back right drive     (was BL) | 18 |
|   c   | Back right CANcoder  (was BL) | 17 |
|   d   | Back right steer     (was BL) | 16 |
|   e   | Intake Intake Motor           | 15 |
|   f   | Intake Extend Aft/Back        | 14 |
|   g   | Transit Motor                 | 13 |
|   h   | Front right drive    (was BR) | 12 |
|   i   | Front right CANcoder (was BR) | 11 |
|   j   | Front right steer    (was BR) | 10 |
|   k   | Shooter motor top             |  9 |
|   l   | (no longer used)              |  - |
|   m   | Shooter motor bottom          |  7 |
|   n   | Front left drive     (was FR) |  6 |
|   o   | Front left CANcoder  (was FR) |  5 |
|   p   | Front left steer     (was FR) |  4 |
|   q   | Intake Extend Front           |  3 |
|   r   | Intake Extend Front CANcoder  | 20 |
|   s   | Back left drive      (was FL) |  2 |
|   t   | Back left CANcoder   (was FL) |  1 |
|   u   | Back left steer      (was FL) |  0 |

## TODO: this needs to be added to the CAN bus

| order | device                       | ID |
| ----- | ---------------------------- | -- |
|       | CANdle                       |    |
