# UBlox GNSS Driver

Driver for the uBlox M* series of GNSS receivers.

## How to use
This driver returns location information as provided by the device. It attempts
do do this for all ublox devices. In the simplest use case, the driver will
continually update a 'location' object, which can be queried by the user as
required.

### Quick start/Automatic
Connect to the device using the provided examples (e.g.  [i2c
example](./examples/i2c.toit) or [serial example](./examples/serial.toit)).
When the driver is instantiated, it:
1. Starts a message receiver task and waits for messages.
2. Attempts to determine hardware version.  If unsuccessful, assumes the device
   is compatible with M8+ protocol instructions.
3. Enables uBlox logging messages, and disables NMEA messages.
4. Subscribes to regular time and position updates from the device.
5. Makes this information available to the user via:
   - `.location` object (this object is `null` until the first fix has been
     made.)
   - `.diagnostic` object (is fulfilled with diagnostic information).
   - `.latest-message["UBX-XXX-YYY"]` map object, which holds the latest message
     the driver has receieved, of any/all `UBX-XXX-YYY` message types.

### Manual Operation
There are other features and information that ublox devices can provide.
Accessing additional information is based on sending messages to and from the
device. The driver allows the user to send/recieve messages to the device
manually.  This is done by:
1. Instantiating the driver, and additionally disabling automatic operation using the
   parameter `--no-auto-run` on the constructor.
2. Manually creating the required messages by `import ubx-message` importing the
   parser library and creating messages.
3. Manually start the message receiver/parser, using `.run`, when desired.
4. Manually send created messages to the device using driver methods, such as
   `send-message`, `send-raw-byte-array` and `send-raw-message`.

> [!WARNING]
> Using this method, any of the implemented message types and/or responses are
> available.  `.location` and `.diagnostic` will have information available if
> subscriptions to the required messages are made, but this is up to the user.
> If the message parser has been started (using `.run`), the latest message
> received of each type will be available in the
> `.latest-message["UBX-XXX-YYY"]` map object.

The information these devices can provide is vast.  As it is a binary protocol,
not all types have human readable output.  Not all message types have yet been
implemented.  It is up to the user to look at the device manual and determine
which messages are required, how to interpret the data, and to construct the
appropriate message types to send to the device (via the driver).

## Interface and Port Support
The uBlox devices support a fixed set of connectivity options, which are
referred to as 'ports' in the uBlox documentation.  Various breakout boards
feature different types of connectivity, not all connectivity options are
available on every device.
> [!WARNING]
> Note: where a particular product only has one interface (port), it is possible
> to disable that port and lock yourself out of the device.

| `PORT` reference | Interface | Driver Support | Device Support |
|-|-|-|-|
| `PORT-UART1` (1)<br>`PORT-UART2` (2) | Serial | Supported. [Serial example](./examples/serial.toit). | `PORT-UART1` common on most devices.  `PORT-UART2` usually not available on M6/7 devices.  Ports may be shared with USB.  Also missing on M9+ 'low power' devices. |
| `PORT-SPI` (4) | SPI | Supported. [SPI example](./examples/spi.toit). | Not common on devices before M8+, and often dropped on 'low-power' devices. |
| `PORT-DDC` (0) | DDC (aka: I2C) | Supported. [I2C example](./examples/i2c.toit). | `DDC` stands for Display Data Channel, but in u-blox receivers it is simply another name for the I2C interface. Commonly available across the board, except on serial-only devices.|
| `PORT-USB` (3) | USB | Partially supported. | Some devices wire this by sharing a UART with USB, and may require that traces are cut on breakouts.  See your modules' datasheet. |
| `PORT-RES5` (5) | RES5 | Reserved. | _Must be treated as invalid, or ignored._ |

## Devices Tested
This driver aims to support all UBX-compatible devices, though coverage is
limited by the hardware and chipsets available to us for testing.  The code was
first written for the M8 device, with others added later.

If you have a device that isn’t listed and can donate one, or help with
testing, please [get in touch](https://discord.gg/Q7Y9VQ5nh2) or open an issue —
contributions and reports are very welcome.

Devices and their reported HW/SW/PROTVER versions are listed:
| Device | HW Version | 'ROM CORE' (SW-VER) | Minumum UBX Protocol Version | From Toit Driver Version | Notes |
|-|-|-|-|-|-|
| M8 | - | - | `15.00`  | >`1.2.0` | Originally written to support this device. |
| `NEO-7M-0-000` | `00070000` | >`1.00 (59842) Jun 27 2012` | `14.0` | `1.3.0` | Operational, undergoing testing. |
| `NEO-6M-0-001` | `00040007` | >`7.03 (45969) Mar 17 2011` | `14.0` (Assumed, device does not advertise  extensions)  | `1.3.0` | Operational, undergoing testing. |

## Protocol Support
Driver uses the [`ubx-message`](https://github.com/toitware/ubx-message)
protocol, parser to handle parsing messages from the ublox device.  The devices
also support other open protocols, which will be developed as needed.
| Protocol | Support | Notes |
|-|-|-|
| [`ubx-message`](https://github.com/toitware/ubx-message) | Yes | Binary, Open but proprietary.  Originally written alongside this driver in support of the M8 device.  Often a subset is supported on **non-ublox** devices. |
| `NMEA` | Not yet. | ASCII text sentences (e.g., GGA, RMC) widely used for basic GNSS output.  Widely supported on ublox and **non-ublox** devices. |
| `RTCM` | Not yet. | Standard for GNSS differential/RTK correction messages. u-blox M8 documentation mentions RTCM, however only specific models feature full support. |
