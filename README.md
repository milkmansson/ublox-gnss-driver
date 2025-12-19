# UBlox GNSS Driver

Driver for the u-blox M* GNSS receivers.

## How to use
This driver returns location information as provided by the device. It attempts do do this for all ublox devices. In the simplest use case, the driver will continually update a 'location' object, which can be queried by the user as required.

### Quick start/Automatic
Connect to the device using the provided examples (e.g.
[i2c example](./examples/i2c.toit) or [serial example](./examples/serial.toit)).  When the driver is instantiated, it:
1. Starts a message receiver task and waits for messages.
2. Attempts to determine hardware version.
3. Enables uBlox logging messages, and disables NMEA messages.
4. Subscribes to regular time and position updates from the device.
5. Makes this information available to the user via:
   - `.location` object (this object is `null` until the first fix has been made.)
   - `.diagnostic` object (is fulfilled with diagnostic information).
   - `.latest-message["UBX-XXX-YYY"]` map object, which holds the latest message the driver has receieved, of any/all `UBX-XXX-YYY` message types.

### Manual Operation
There are other features and information that ublox devices can provide.
Accessing additional information is based on sending messages to and from the
device. The driver allows the user to send/recieve messages to the device manually.  This is done by:
1. Instantiating the driver, using the parameter `--no-auto-run`.
2. Manually creating the required messages using the `ubx-message` parser package.
3. Manually sending those to the device using driver functions such as `


The 'Advanced examples' below show how to use the mesage system via this driver to
collect information from any of the implemented message types. The information
they can provide is vast, and not all types may have human readable output. It
is up to the user to look at the device manual and determine which messages are
required, how to interpret the data, and to construct the message to send via
the driver to ask for it.
