### Note

In the topic published by the metal detector node, the channel field of the Coil message type holds the measurements for both coils. Considering a posterior view of the robot (from behind), the measurements correspond to:

* channel[0] -> Left coil
* channel[1] -> Right coil

The coils measurements are read one at a time, and switching between coils occurs in a fixed interval.