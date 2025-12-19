// Copyright (C) 2025 Toit Contributors. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import .diagnostics
import gnss-location show GnssLocation
import io
import location show Location
import log
import monitor
import reader as old-reader
import serial
import ubx-message

import .reader
import .writer


/**
Driver for u-blox GNSS devices.

Originally developed for the Max M8 GPS module.
*/

// For backward compatibility.
I2C-ADDRESS ::= 0x42

class Driver:
  static I2C-ADDRESS ::= 0x42

  static METER-TO-MILLIMETER ::= 1000
  static METER-TO-CENTIMETER ::= 100
  static COORDINATE-FACTOR/float ::= 10_000_000.0
  static QUALITY-SAT-COUNT_ ::= 4

  // Maximum ms to wait for command latches.
  static COMMAND-TIMEOUT-MS_ ::= 5000

  // NMEA Helpers while a protocol specific parser doesn't exist.
  // See $disable-nmea-messages.
  static NMEA-CLASS-ID_ := 0xF0
  static NMEA-MESSAGE-IDS_ := {
    "GGA": 0x00,
    "GLL": 0x01,
    "GSA": 0x02,
    "GSV": 0x03,
    "RMC": 0x04,
    "VTG": 0x05,
    "GRS": 0x06,
    "GST": 0x07,
    "ZDA": 0x08,
    "GBS": 0x09,
    "DTM": 0x0A,
  }

  time-to-first-fix_/Duration := Duration.ZERO

  // Latches/Mutexes for managing and acknowledging commands
  waiters-latch_ := []             // Stores latches for all users until fix.
  waiters-mutex_ := monitor.Mutex  // Prevent race when $waiters_ is read/written.
  command-mutex_ := monitor.Mutex  // Used to ensure one command at once.
  command-latch_ := monitor.Latch  // Used to ensure cfg gets the result.

  // Loggers - one for driver, and separate one for UBX device sourced messages.
  logger_/log.Logger := ?
  ubx-logger_/log.Logger := ?

  // Container for the message receiver task.
  runner_/Task? := null

  // Objects for return data.
  diagnostics_/Diagnostics := Diagnostics --known-satellites=0 --satellites-in-view=0 --signal-quality=0.0 --time-to-first-fix=Duration.ZERO
  location_/GnssLocation? := null
  adapter_/Adapter_ := ?
  device-protocol-version_/string? := null

  // Map to contain the most recent message of every given type.
  latest-message/Map := {:}

  // HWVERSION to expected protver lookup.
  // Done this such that users can insert their own HWVERSION if needed.
  ublox-protversion-lookup/Map := {
    "00070000":"14.00",       // M7
    "00040007":"13.00",       // M6
  }

  /**
  Creates a new driver object.

  The $reader should be an $io.Reader, but $old-reader.Reader objects are
    still supported for backwards compatibility. Support for $old-reader.Reader
    is deprecated and will be removed in a future release.
  Use $Reader to create an $io.Reader from a $serial.Device.

  The $writer should be an $io.Writer, but "old-style" writers are still
    supported for backwards compatibility. Support for "old-style" writers is
    deprecated and will be removed in a future release.
  Use $Writer to create an $io.Writer from a $serial.Device.

  When starting the driver with defaults, the driver subscribes to the messages
    required to find location and time.  For advanced users looking to prevent
    the default operation, and create subscriptions/messge handlers, etc,
    themselves, specify `--no-auto-run` on the constructor.  In this case,
    users must set required configurations (via $send-message-cfg), subscribe
    to desired message types (via $send-set-message-rate), and then start
    the message receiver task ($run) manually.
  */

  constructor reader writer logger=log.default
      --auto-run/bool=true
      --force-protocol-version/string?=null
      --hw-reset/bool=true:
    logger_ = logger.with-name "ublox-gnss"
    ubx-logger_ = logger.with-name "ubx"

    if reader is old-reader.Reader:
      reader = io.Reader.adapt reader

    if writer is not io.Writer:
      writer = io.Writer.adapt writer

    adapter_ = Adapter_ reader writer logger

    // Hardware reset - useful if running multiple tests without power reset.
    if hw-reset:
      reset --mode=4

    if auto-run:
      // Start message receiver task (and wait for it to start).
      run

      // Start debug messages in UBX firmware:
      enable-inf-messages

      // Get device type info, before sending any commands.
      detect-device-version

      // Turn off default (unused) NMEA messages.
      disable-nmea-messages

      // If forced, manually set device protocol version.
      if force-protocol-version:
        logger_.debug "Protocol version forced." --tags={"detected": device-protocol-version_, "forced": force-protocol-version}
        device-protocol-version_ = force-protocol-version

      // Start subscription to default messages.
      start-periodic-nav-packets_

  /**
  Returns the time-to-first-fix.

  This value refers to the first fix.  If the fix is lost and found again, this
    value does not change.  (This is by design.)
  */
  time-to-first-fix -> Duration: return time-to-first-fix_

  /**
  Returns a diagnostic object with various diagnostic values.
  */
  diagnostics -> Diagnostics: return diagnostics_

  /**
  Returns current location, or null if no fix yet.

  If a fix is made, but is lost, this value remains the location of the last
    fix, and does not change back to null. (This is by design.)
  */
  location -> GnssLocation?:
    return location_

  /**
  Variant of $location, which blocks until a fix.
  */
  location --blocking -> GnssLocation:
    latch := monitor.Latch
    waiters-mutex_.do:
      waiters-latch_.add latch
    return latch.get

  /**
  Starts the message receiver task.

  The $run command returns only when the task has started.  This ensures the
    message receiver is ready before any messages are sent that would otherwise
    cause code to block permanently without the corresponding ACK/NAK being
    received.
  */
  run -> none:
    assert: not runner_
    adapter_.flush
    start-latch := monitor.Latch
    duration := Duration.of:
      // Start the message parser task to parse messages as they arrive.
      runner_ = task::
        start-latch.set true
        while true:
          message := adapter_.next-message

          // Store latest version of messages for other handlers to use.
          latest-message[message.full-name] = message

          if message is ubx-message.AckAck:
            process-ack-ack_ message as ubx-message.AckAck
          else if message is ubx-message.AckNak:
            process-ack-nak_ message as ubx-message.AckNak

          else if message is ubx-message.CfgInf:
            process-cfg-inf_ message as ubx-message.CfgInf
          else if message is ubx-message.CfgMsg:
            process-cfg-msg_ message as ubx-message.CfgMsg
          else if message is ubx-message.MonVer:
            process-mon-ver_ message as ubx-message.MonVer

          else if message is ubx-message.NavSvInfo:
            process-nav-svinfo_ message as ubx-message.NavSvInfo
          else if message is ubx-message.NavSol:
            process-nav-sol_ message as ubx-message.NavSol
          else if message is ubx-message.NavPosLlh:
            process-nav-posllh_ message as ubx-message.NavPosLlh

          else if message is ubx-message.NavStatus:
            process-nav-status_ message as ubx-message.NavStatus
          else if message is ubx-message.NavPvt:
            process-nav-pvt_ message as ubx-message.NavPvt
          else if message is ubx-message.NavSat:
            process-nav-sat_ message as ubx-message.NavSat

          else if message is ubx-message.NavTimeUtc:
            process-nav-timeutc_ message as ubx-message.NavTimeUtc

          // UBX Software Messages will just be displayed.
          else if message is ubx-message.InfDebug:
            ubx-logger_.debug (message as ubx-message.InfDebug).text
          else if message is ubx-message.InfError:
            ubx-logger_.error (message as ubx-message.InfError).text
          else if message is ubx-message.InfNotice:
            ubx-logger_.info (message as ubx-message.InfNotice).text
          else if message is ubx-message.InfTest:
            ubx-logger_.info (message as ubx-message.InfTest).text
          else if message is ubx-message.InfWarning:
            ubx-logger_.warn (message as ubx-message.InfWarning).text

          else:
            logger_.debug  "Driver received UNHANDLED message type: $message"

    start-latch.get
    logger_.debug "message receiver started" --tags={"ms": duration.in-ms}

  /**
  Resets the driver.

  Reset should be called when the message receiver is not actively running,
    otherwise some messages will be lost.  (may or may not be a problem
    depending on the use case.)

  Set $mode to 1 (default) "Controlled Software Reset" restart the software
    but fix and satellite information is not lost. Set $mode to 4 for a hardware
    restart via watchdog after a shutdown (loses fix and tracking information).
  */
  reset --mode/int=1 -> none:
    logger_.debug "sending reset message (type $mode)"
    adapter_.reset --mode=mode
    time-to-first-fix_ = Duration.ZERO
    location_ = null
    sleep --ms=200

  /**
  Stops the message receiver task and shuts down the adapter.
  */
  close -> none:
    if runner_:
      runner_.cancel
      runner_ = null

  /**
  Processor for recieved UBX-ACK-NAK messages.

  Message acknowledges that the sent CFG/poll/etc command didn't work.  Reasons
    are not given.  Some troubleshooting information may be obtained by enabling
    ublox logging (UBX-INF-*) message types.
  */
  process-ack-nak_ message/ubx-message.AckNak -> none:
    command-latch_.set (message as ubx-message.AckNak)

  /** Processor for recieved UBX-ACK-ACK messages. */
  process-ack-ack_ message/ubx-message.AckAck -> none:
    command-latch_.set (message as ubx-message.AckAck)

  /** Processor for recieved UBX-CFG-INF messages. */
  process-cfg-inf_ message/ubx-message.CfgInf -> none:
    command-latch_.set (message as ubx-message.CfgInf)

  /** Processor for recieved UBX-CFG-MSG messages. */
  process-cfg-msg_ message/ubx-message.CfgMsg -> none:
    command-latch_.set (message as ubx-message.CfgMsg)

  /** Processor for recieved UBX-MON-VER messages. */
  process-mon-ver_ message/ubx-message.MonVer -> none:
    command-latch_.set (message as ubx-message.MonVer)

  /** Processor for recieved UBX-NAV-SOL messages. */
  process-nav-sol_ message/ubx-message.NavSol -> none:
    //logger_.debug "Received NavSol message." --tags={"position-dop" : message.position-dop} // , "longitude" : message.latitude-deg, "itow": message.itow }

  /** Processor for recieved UBX-NAV-TIMEUTC messages. */
  process-nav-timeutc_ message/ubx-message.NavTimeUtc -> none:
    //logger_.debug "Received NavTimeUtc message." --tags={"valid-utc" : message.valid-utc, "time-utc": message.utc-time }

  /** Processor for recieved UBX-NAV-STATUS messages. */
  process-nav-status_ message/ubx-message.NavStatus -> none:
    //logger_.debug "Received NavStatus message." --tags={"fix":message.gps-fix-text,"ttff-ms": message.time-to-first-fix }

    if time-to-first-fix_.in-ns != 0: return
    time-to-first-fix_ = Duration --ms=message.time-to-first-fix

  /**
  Processor for recieved UBX-NAV-PVT position messages. (M8+)

  Data obtained by this message is available in object.Location.  Message in
    its raw format is available in $latest-message["UBX-NAV-PVT"].
  */
  process-nav-pvt_ message/ubx-message.NavPvt -> none:
    if message.is-gnss-fix:
      location_ = GnssLocation
        Location message.lat / COORDINATE-FACTOR message.lon / COORDINATE-FACTOR
        message.height-msl.to-float / METER-TO-MILLIMETER
        message.utc-time
        message.horizontal-acc.to-float / METER-TO-MILLIMETER
        message.vertical-acc.to-float / METER-TO-MILLIMETER

      waiters-mutex_.do:
        waiters := waiters-latch_
        waiters-latch_ = []
        waiters.do:
          it.set location_

  /**
  Processor for recieved UBX-NAV-POSLLH position messages. (M6/M7)

  Data obtained by this message is available in object.Location.  Message in
    its raw format is available in $latest-message["UBX-NAV-POSLLH"].
  */
  process-nav-posllh_ message/ubx-message.NavPosLlh:
    //logger_.debug "Received NavPosLlh message." --tags={"latitude" : message.latitude-deg , "longitude" : message.longitude-deg, "itow": message.itow }

    // This message type doesn't natively contain time or fix status like the
    // UBX-NAV-PVT messages. Instead, we will make the code non breaking by
    // filling the existing structure with information from latest recieved of
    // other message types.
    status-message/ubx-message.NavStatus? := null
    if latest-message.contains "UBX-NAV-STATUS":
      status-message = latest-message["UBX-NAV-STATUS"]
    time-message/ubx-message.NavTimeUtc? := null
    if latest-message.contains "UBX-NAV-TIMEUTC":
      time-message = latest-message["UBX-NAV-TIMEUTC"]

    // If fix is right/enough, give the data back to the destination object.
    // Yes there is a risk that the fix could stop, and the location data
    // become stale, however we need breaking changes to old and new code.
    // (This matches pre-existing functionality (eg non breaking).  We can do
    // in a later PR.  THUS: Returning a location depends on all three messages.
    if status-message and time-message:
      if status-message.gps-fix >= ubx-message.NavStatus.FIX-3D:
        location_ = GnssLocation
          Location message.latitude-raw / COORDINATE-FACTOR message.longitude-raw / COORDINATE-FACTOR
          message.height-msl-mm.to-float / METER-TO-MILLIMETER
          time-message.utc-time
          message.horizontal-accuracy-mm.to-float / METER-TO-MILLIMETER
          message.vertical-accuracy-mm.to-float / METER-TO-MILLIMETER

        waiters-mutex_.do:
          waiters := waiters-latch_
          waiters-latch_ = []
          waiters.do:
            it.set location_

  /**
  Processor for recieved UBX-NAV-SAT satellite info messages. (M8+)

  This message type is used to produce diagnostic data to give information such
    as fix type, signal quality, etc.  Data obtained by this message is
    available in object.diagnostics.  Message in its raw format is available
    in $latest-message["UBX-NAV-SAT"].
  */
  process-nav-sat_ message/ubx-message.NavSat -> none:
    cnos ::= []
    satellite-count ::= message.num-svs
    satellite-count.repeat: | index |
      satellite-data ::= message.satellite-data index
      cnos.add satellite-data.cno

    cnos.sort --in-place: | a b | b - a
    n ::= min cnos.size QUALITY-SAT-COUNT_
    sum := 0.0
    n.repeat: sum += cnos[it]
    quality ::= sum / QUALITY-SAT-COUNT_

    satellites-in-view := cnos.reduce --initial=0: | count cno |
      count + (cno > 0 ? 1 : 0)
    known-satellites := satellite-count
    diagnostics_ = Diagnostics
        --time-to-first-fix=time-to-first-fix
        --signal-quality=quality
        --satellites-in-view=satellites-in-view
        --known-satellites=known-satellites

  /**
  Processor for recieved UBX-NAV-SAT satellite info messages. (<=M7) */
  process-nav-svinfo_ message/ubx-message.NavSvInfo:
    cnos ::= []
    satellite-count ::= message.satellite-count
    satellite-count.repeat: | index |
      satellite-data ::= message.satellite-data index
      cnos.add satellite-data.cno

    cnos.sort --in-place: | a b | b - a
    n ::= min cnos.size QUALITY-SAT-COUNT_
    sum := 0.0
    n.repeat: sum += cnos[it]
    quality ::= sum / QUALITY-SAT-COUNT_

    satellites-in-view := cnos.reduce --initial=0: | count cno |
      count + (cno > 0 ? 1 : 0)
    known-satellites := satellite-count
    diagnostics_ = Diagnostics
        --time-to-first-fix=time-to-first-fix_
        --signal-quality=quality
        --satellites-in-view=satellites-in-view
        --known-satellites=known-satellites

  /**
  Sends subscriptions for required messages to discover current location.

  Needs to understand what device is currently configured so that the right
    generation of messages can be requested.  Otherwise will fall back to
    orignal design version (M8).

  If using --disable-auto-run in the constructor, --force-protocol-version must
    also be used to ensure the correct protocol version and prevent fallback.
  */
  start-periodic-nav-packets_ -> none:
    // Request UBX-NAV-TIMEUTC packets to give time info, every 15 sec.
    set-message-rate_ ubx-message.Message.NAV ubx-message.NavTimeUtc.ID 15

    // Request UBX-NAV-STATUS packets to give diagnostic info, every sec.
    set-message-rate_ ubx-message.Message.NAV ubx-message.NavStatus.ID 1

    // Using a float until better option (semver?)
    prot-ver := float.parse device-protocol-version_

    if prot-ver >= 15.0 :
      logger_.debug "Setting up for M8+ device type."
      // Request NAV Position/Velocity/Time message every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavPvt.ID 1
      // Request NAV Satellite tracking messages every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavSat.ID 1

    else if prot-ver >= 14.0:
      logger_.debug "Setting up for 7M device type (legacy)."
      // Request Geodetic Position Solution messages every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavPosLlh.ID 1  // Legacy Equivalent to NavPvt Messages
      // Request Satellite tracking messages every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavSvInfo.ID 1  // Legacy Equivalent to NavSat Messages
      // Request position, velocity and time (in ECEF) messages every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavSol.ID 1

    else if prot-ver >= 13.0:
      logger_.debug "Setting up for 6M device type (legacy)."
      // Request Geodetic Position Solution messages every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavPosLlh.ID 1  // Legacy Equivalent to NavPvt Messages
      // Request Satellite tracking messages every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavSvInfo.ID 1  // Legacy Equivalent to NavSat Messages
      // Request position, velocity and time (in ECEF) messages every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavSol.ID 1

    else:
      // Assume all others are M8 or later (for now):
      logger_.debug "Defaulting to M8 Device Type."
      // Request NAV Position/Velocity/Time message every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavPvt.ID 1
      // Request NAV Satellite tracking message every 1 second.
      set-message-rate_ ubx-message.Message.NAV ubx-message.NavSat.ID 1

  /**
  Sends a subscription for given $message-id, and $rate.

  Sends a UBX-CFG message asking for the specified $class-id/$message-id
    message type to be sent at the given $rate.

  If $port not given, rate is set for all ports.
  */
  set-message-rate_ class-id message-id rate --port/int=-1 -> none:
    // Poll for a UBX-CFG-MSG
    message := ubx-message.CfgMsg.poll --msg-class=class-id --msg-id=message-id
    cfg-message := (send-message message) as ubx-message.CfgMsg

    // For the specific port type, set the rate.
    cfg-message.set-rate port --rate=rate

    // Send the adjusted UBX-CFG-MSG message back to set the configuration.
    cfg-response := send-message cfg-message

    // Report only if failure (NAK) or unexpected response type.
    if cfg-response is ubx-message.AckNak:
      logger_.debug "send-set-message-rate_ FAILURE" --tags={"message":"$message","response":"$cfg-response"}
    else if not (cfg-response is ubx-message.AckAck):
      logger_.debug "send-set-message-rate_ UNEXPECTED" --tags={"message":"$message", "response":"$cfg-response"}

  /**
  Sends a request for device information, using UBX-MON-VER message.

  Message in its raw format is available in $latest-message["UBX-MON-VER"].
  */
  detect-device-version -> none:
    //logger_.debug "Send Version Request Poll."
    message := ubx-message.MonVer.poll
    ver-message := (send-message message) as ubx-message.MonVer

    // Determine supported version number and set it in the private variable.
    device-protocol-version := supported-protocol-version ver-message
    device-protocol-version_ = device-protocol-version

    logger_.debug "received MON-VER message" --tags={
      "sw-ver": ver-message.sw-version,
      "hw-ver": ver-message.hw-version,
      "determined-prot-ver": device-protocol-version}


  /**
  Enables UBX debugging messages, using UBX-CFG-INF message.

  The latest received message of each UBX-INF-xxxx type is available in its raw
    format in $latest-message["UBX-INF-xxxx"].
  */
  enable-inf-messages -> none:
    //logger_.debug "Send UBX-CFG-INF poll (Enable INFO, all outputs)."
    message := ubx-message.CfgInf.poll
    cfg-message := (send-message message) as ubx-message.CfgInf

    // Now we have a returned UBX-CFG-INF message, adjust it and send it back.
    cfg-message.enable-all

    // Send adjusted CFG-INF message back to set the configuration.
    cfg-response := send-message cfg-message

    // Determine and display the result.
    if cfg-response is ubx-message.AckAck:
      logger_.debug "enabled ubx device debug messages"
    else if cfg-response is ubx-message.AckNak:
      logger_.debug "FAILED to enable ubx device debug messages" --tags={"message":"$cfg-response"}
    else:
      logger_.debug "UNEXPECTED enabling ubx debug device messages" --tags={"message":"$cfg-response"}

  /** Send a raw byte array to the device, for debug purposes. */
  send-raw-byte-array byte-array/ByteArray -> none:
    logger_.debug "sending raw bytearray: $byte-array"
    command-mutex_.do:
      adapter_.send-packet byte-array

  /** Send a user created message to the device, for debug purposes. */
  send-raw-message message/ubx-message.Message -> none:
    logger_.debug "sending custom message: $message"
    command-mutex_.do:
      adapter_.send-packet message.to-byte-array

  /**
  Sends message, and waits for the response.

  Handles logic of success and failure messages, while not blocking other
    message traffic being handled by the driver.  Note that new/custom message
    types being sent may require latch handling to avoid always being handled
    via the $COMMAND-TIMEOUT-MS_ timeout path, and to catch the relevant message
    that matches the command.
  */
  send-message message/ubx-message.Message --return-immediately/bool=false -> ubx-message.Message?:
    response := message
    command-mutex_.do:
      if return-immediately:
        adapter_.send-packet message.to-byte-array
        return null

      // Reset the latch to prevent stray ACK/NAK getting used.
      command-latch_ = monitor.Latch

      // todo: try/finally.
      // todo: determine if/how we should convert to semphore.

      duration := Duration.ZERO
      exception := catch:
        with-timeout --ms=COMMAND-TIMEOUT-MS_:
          duration = Duration.of:
            adapter_.send-packet message.to-byte-array
            response = command-latch_.get

      // Set latch to null if we're not using it.
      command-latch_ = null

      if exception:
        logger_.error "Command timed out. " --tags={"message":"$(message)", "ms":duration.in-ms}
        return null

    // Lets have the return message supplied back to the caller to determine
    // what to do with it.
    return response

  /**
  Determines the protocol version supported from the supplied UBX-MON-VER message.

  Makes assumptions if the device doesn't return the protocol version
    explicitly, in accordance with the table in README.md.  Optionally sets the
    class wide property, in case the user decides to specify it manually.
  */
  supported-protocol-version message/ubx-message.MonVer -> string:
    // Find an extension containing PROTVER and if exists parse it
    protver-ext/string? := message.extension "PROTVER"

    if protver-ext != null:
      // Protver exists, parse it.  Some devices delimit by ' ', some devices '=':
      protver-ext = protver-ext.trim
      pos-eq/int := protver-ext.index-of "="
      pos-sp/int := protver-ext.index-of " "
      if pos-eq > -1:
        return protver-ext[(pos-eq + 1)..]
      else if pos-sp > -1:
        return protver-ext[(pos-sp + 1)..]
      else:
        throw "Couldn't parse protver string: '$(protver-ext)'"

    // Use lookup if the protver string doesn't exist in the message:
    if ublox-protversion-lookup.contains message.hw-version:
      return ublox-protversion-lookup[message.hw-version]

    // All else fails = Assume 12. Address later if an issue is raised.
    return "12.00"

  /**
  Disables all default NMEA messages.

  When a Ublox device is turned on, several NMEA messages arrive by default.
    This command iterates through the set of known default messages and sets the
    rate for each to zero, and for all outputs.  Done this way until enough of
    an NMEA Parser is completed to make this useful.

  Note: UBX-CFG-MSG is a legacy method.  Currently supported by later devices
    but could/should use UBX-CFG-VALSET at some later point.  This function uses
    Per-Port method because some outputs still send on all ports.  This function
    does NOT SAVE this configuration to the device.


  This is necessary to quieten the uart as much as possible, increasing
    accuracy for things like time synchronisation.
  */
  disable-nmea-messages -> none:
    // Bytearray gives zero rate for all outputs.
    rates := #[0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    logger_.debug "disabling default NMEA messages"
    NMEA-MESSAGE-IDS_.values.do:
      message := ubx-message.CfgMsg.per-port --msg-class=NMEA-CLASS-ID_ --msg-id=it --rates=rates
      cfg-response := send-message message

      // Report only if failure or unexpected.
      if cfg-response is ubx-message.AckNak:
        logger_.debug "disable NMEA FAILURE" --tags={"msg-id": it,"message":"$cfg-response"}
      else if not (cfg-response is ubx-message.AckAck):
        logger_.debug "disable NMEA UNEXPECTED" --tags={"msg-id": it,"message":"$cfg-response"}


class Adapter_:
  static STREAM-DELAY_ ::= Duration --ms=1

  logger_/log.Logger
  reader_/io.Reader
  writer_/io.Writer

  constructor .reader_ .writer_ .logger_:

  flush -> none:
    // Flush all data up to this point.
    wait-until-receiver-available_

  reset --mode/int=1 -> none:
    wait-until-receiver-available_
    // Reset and reload configuration (cold boot + reboot of processes).
    send-packet (ubx-message.CfgRst --reset-mode=mode).to-byte-array
    // Wait for the reload to take effect, before flushing stale data.
    // This was tested with 10ms, so using 50ms.
    sleep --ms=50
    flush

  send-packet bytes/ByteArray -> none:
    writer_.write bytes
    sleep STREAM-DELAY_

  send-ubx message/ubx-message.Message -> none:
    writer_.write message.to-byte-array
    sleep STREAM-DELAY_

  next-message -> ubx-message.Message:
    while true:
      peek ::= reader_.peek-byte 0
      if peek == 0xb5: // UBX protocol
        e := catch: return ubx-message.Message.from-reader reader_
        log.warn "error parsing ubx message" --tags={"error": e}
      // Go to next byte.
      reader_.skip 1

  wait-until-receiver-available_:
    // Block until we can read from the device.
    first ::= reader_.read

    // Consume all data from the device before continuing (without blocking).
    while true:
      e := catch:
        with-timeout --ms=0:
          reader_.read
      if e: return
