# UBlox GNSS Driver

Driver for the u-blox M* GNSS receivers.

## How to use
This driver returns location information as provided by the device. It attempts do do this for all ublox devices. In the simplest use case, the driver will continually update a 'location' object, which can be queried by the user as required.

### Quick start/Automatic
Connect to the device using the provided [i2c](./examples/i2c.toit) or
[serial](./examples/serial.toit) examples.  When the driver is instantiated, it:
1. Attempts to determine hardware version.
2. Subscribes to regular time and position updates from the device.
3. makes this information available via:
   - `.location` object
   - `.diagnostic` object


There are other features and information that these devices can provide. Accessing additional information is based on sending messages to and from the device. The driver allows the user to send/recieve messages to the device. The 'Advanced examples' below show how to use the mesage system via this driver to collect information from any of the implemented message types. The information they can provide is vast, and not all types may have human readable output. It is up to the user to look at the device manual and determine which messages are required, how to interpret the data, and to construct the message to send via the driver to ask for it.
