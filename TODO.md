

- Make sure the master code sends something every 11ms
-- channel data for real, but internal data for the simulator (add config for this)
-- This means the master code needs to run every 11ms/5.5ms depending in the baud rate

- Check if we respect the 2chars of idle time between packets

- Remove the master examples

- Build a sniffer that works with usb serial
