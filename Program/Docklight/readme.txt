Docklight / Docklight Scripting V2.2.8

Copyright 2016 Flachmann und Heggelbacher GbR (www.fuh-edv.de)

Program Description:
Docklight is a test, analysis and simulation tool for serial communication 
protocols (RS232, RS485/422 and others). It allows you to monitor the 
communication between two serial devices or to test the serial communication
of a single device. Docklight is easy to use and runs on almost any standard 
PC using Windows 10, Windows 8, Windows 7 or Windows XP operating system.

Docklight Scripting provides an easy-to-use program language and a built-in 
editor to create and run automated test jobs. In addition to COM ports (RS232, ...), 
Docklight Scripting supports TCP, UDP, USB HID or Named Pipes.

Installation:
- Run DocklightSetup.exe (resp. DocklightScriptingSetup.exe)

Distribution:
- See the fuh_distribute_support_int.txt" file

Version History:

Docklight / Docklight Scripting V2.2.8 (11/2016)
Fixes and Improvements:
- Windows visual style and high DPI awareness. (No "blurry fonts" with display
  text size 125% or higher.)

Docklight / Docklight Scripting V2.2.4 (07/2016)

New features and functions:
- Scripting: "_auto.pts" script convention for automatic script loading & execution along with the 
  project file.
- Scripting: Extended USBHID channnel options for variable Output Report IDs and size.
- Send and Receive Sequence List have now a small lock to protect against accidental 
  sequence list reordering.
- Special educational and promotional license key scheme combining a license key and the 
  license holder's name.
- License registration dialog now allows deleting/uninstalling the license.
    
Fixes and Improvements:
- Scripting: USB HID compatibility in Windows 10 fixed. USB HID is now "Shared Access".
- Scripting: USB HID now correctly indicates when a TX / write output report did not succeed.
- Mouse wheel support in sequence lists.
- Keyboard Console now supports ESC key to send ASCII code 27 (Escape).
- Improved Checksum spec parser and added example for common LRC. "-MOD256" specifier 
  for "data + checksum = 0" style checksum.
- Fixed bug: On some serial devices with more than 4K serial buffers and high data load, 
  Docklight could crash with "Internal error 1000 in module 
  MdCommunication.printAndLogIntCommData Internal comm. processing error".
- Fixed bug: When using a certain sequence of opening, closing and re-opening the 
  Control Character Shortcuts dialog, Docklight could crash. 
- Fixed bug: RTS line corrected for Flow Control "RS485 Transceiver Control"
  and "Hardware Handshaking".
- Fixed bug: Keyboard Console line break behavior, now CR/LF (or CR only or LF only) 
  is always added correctly. 

Docklight / Docklight Scripting V2.1.10 (09/2015)

Note: 
- New V2.1 project file format for checksum support. 
  Projects without checksums are stored in the V2.0 format for compatibility. 

Fixes and Improvements: 
- Faster RTF (colored text) formatting, especially with text that contains a lot of color changes. 
- More readable and more up-to-date fonts and layout.
- Improved display overload handling, e.g. when RTF formatting still takes too long. Docklight does
  not freeze, but switch to Plain Text Mode automatically and indicate a warning. 
- Improved COM port enumeration in the Project Settings dialog: 
  Should find some rare device types that were previously omitted. Uses now SetupAPI, plus additional 
  registry readout from HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM. 
- Added Receive Sequence comment macros: %_N for Sequence Name, %_I for Sequence Index,
  %_X for data direction.
- Adjusted standard baud rate list, includes now 230400 baud.
- Fixed bug: For "RS485 Transceiver Control - Set RTS high while sending", XON/XOFF was enabled, too. 
- Fixed bug: Tap Pro time drift correction with Windows Time / fixed "jump to earlier time"
- Fixed bug: Tap Pro baud rates 230.4K and higher, ParityReplacementChar = 0 (ignore)
- Fixed bug: Bitwise comparison operator ('^') would not work after '#' wildcard section. 
- Fixed RTS/DTR, XON/XOFF inconsistencies: No temporary RTS/DTR glitches when switching baud rates
  or opening port. Corrected XON/XOFF when project setting and not script controlled.
- Ctrl+W (Clear Communications) works now regardless of focus
- Fixed bug: "Repeat Send" button state wrong after switching representation
- Fixed bug: Sequence lists: Delete a row, then use up/down keys - wrong line was marked
- Fixed bug: Rare crash when using Find for Communication Window
- Scripting: Changed name and extended DL.GetEnvironment() function. Added Docklight-specific keywords
  for reading Docklight project and version data.
- Scripting: Active Keyboard Console now blocks/disables script editor text box.
- Scripting: Fixed crash when starting with command line option "-r", then closing with Alt+F4. 
- Scripting: Fixed #include bug: "Script preprocessor error: Duplicate Include for same file" appeared,
  even for unique include paths. 
- Scripting: Fixed long script loading times for large scripts (> 100KB).
- Scripting: ConvertSequenceData - new "fromText" argument to convert plain text 
  to a HEX/Dec/Binary sequence string

New features and functions:
- Checksum calculation (CRC and others) for Send and Receive sequences.
- 'Standard' Docklight now includes Function Characters support (Handshake Monitoring & Control)
- 'Standard' Docklight now includes Docklight Tap Pro / 485 support
- Channel Alias setting for custom data direction labels
- Scripting: New DL.GetReceiveComments() method - chronological list of all receive sequence comments.
- Scripting: New GetCommWindowData() method - evaluate the original comm.window output in your script
- Scripting: USB HID and Named Pipes support
- Scripting: "PROXY:<tcpPort>" - TCP Server that forwards the client's connect/disconnect behavior 
- Scripting: DL.InputBox2 for a InputBox that appears on the same screen as the Docklight window
- Scripting: FileInput/FileOutput additions: .Dialog(...), .FileExists(...)
  See the \ScriptSamples\Extras\FileDialog_Example folder for an example.  


Docklight / Docklight Scripting V2.0.5 (02/2013)
New features and functions: 
- New script command "DL.SetHandshakeSignals rts As Boolean, dtr As Boolean", e.g.
  DL.SetHandshakeSignals true, false
Fixes and Improvements: 
- Improved COM port enumeration for the Project Settings dialog. Fixed freeze / very
  slow enumeration on some recent systems, additional port descriptions.
- Fixed issue with program shutdown behavior (e.g. on program close through 
  Task Manager or setup.exe). 
- In "Flow Control = Manual" you can change RTS/DTR states now without opening the 
  COM port. A "Flow Control = Manual" project now always start with RTS=low and 
  DTR = low
- DL.SetChannelSettings now allow changing baud rates and other serial parameters 
  on-the-fly, i.e. without closing the communication channel first. 
- Fixed v2.0.1 bug: Communication Filter not correctly maintained after a project
  was loaded via DL.OpenProject

Docklight / Docklight Scripting V2.0.1 (08/2012)
New features and functions: 
- Support for EZ-Tap Pro and Versa-Tap from www.stratusengineering.com
  (see also http://docklight.de/support/support_dl_faq034.htm) 
- New setting for COM communication channels: 
  Baud Rate "None" = Don't set serial communication settings (baud rate, parity ...) 
  (to avoid problems with virtual COM drivers / Embedded COM stacks that do not support this)
- New OnReceive_GetDateTime() / OnReceive_GetMilliseconds() method
- New DL.ConvertSequenceData() for processing float and integer encoded values
- New DL.SetContentsFilter() to change Communication Filter settings while channel is open
- New Function Character '^' for bitwise comparisons
- New Receive Sequence comment macros (%_L, %_T, %_C, %_A, %_H, %_D, %_B)
- New method DL.GetHandshakeSignals()
- New method DL.GetEnvironmentVariable(<environmentString>), e.g. DL.GetEnvironmentVariable("USERNAME")

Fixes and Improvements:
- Improved Receive Sequence matching: no more restrictions cursor-based matching. All possible
  matches are considered. 
- Improved non-modal "Find Sequence..." dialog. Better matching, wildcards now allowed
- New command line option '-i' for invisible background operation
- Extended OnSend_GetData() / OnReceive_GetData() syntax for returning substrings
- Fixed bug with StartLogging: "Internal error 1000 in module MdCommunication.printAndLogIntCommData"
- Fixed ghost ".exe" when closing Docklight (using Alt + F4) while logging is active
- Fixed crash in "Create Log file(s)" when providing an illegal base file path 
- Fixed bug in File->Import Sequence List..: Receive Action index was incorrectly set.
- Docklight allows now manually editing a project file in a text editor, without having 
  to maintaining the original sequence indexes. Docklight will re-index on project load 
- Digitally signed setup packages and executables (.exe, .dll)


Docklight / Docklight Scripting V1.9.21 (05/2009)
New features and functions: 
- Support for Docklight Monitoring Tap USB - Choose comm.channels TAP0 / TAP1
  (e.g.) instead of COM4 / COM5 for best monitoring accuracy better than 10 
  milliseconds
- Milliseconds timestamp resolution and improved overall timing accuracy
  (menu Tools-->Options, "Date/Time Stamps")
- Menu Tools --> Expert Options: 
  Comm. Driver Mode = External / High Priority Process
  for higher accuracy when monitoring serial COM ports
- Menu Tools --> Expert Options: Disable data forwarding in Monitoring Mode 
  for higher data throughput when monitoring serial COM ports
- Receive Sequence Function Character "!" to detect handshake signal changes
- Menu Scripting --> Customize / External Editor: 
  use a third party program editor with advanced editing features like 
  syntax highlighting
- Break Script menu/toolbar and DL.Break method: 
  interrupts script execution and shows the current line in the editor
- #include statement / include file support for scripts
 
Fixes and Improvements:
- Keyboard console state is now remembered when communication is stopped and 
  started again.
- Flow Control Support "Manual" now displays the RI (Ring Indicator) status, too
- Multiple input / output files: 
  Up to 4 FileInput objects and 4 FileOutput objects can be used simultaneously
  FileInput, FileInput2, FileInput3, FileInput4
  FileOutput, FileOutput2, FileOutput3, FileOutput4
- Show recently used network addresses in the drop down list of the 
  Project Settings dialog
- DL.AddComment Chr(7) can be used to produce a beep sound 
- You can now call user-defined Subs or Functions from inside DL_OnSend() or
  DL_OnReceive()
- CRCs with less than 7 bits now available. New predfined keyword "CRC-7" for
  7 bit CRC used with MMC / smart card applications 
- Improved performance when opening / starting large scripts
- Fixed bug: Flow control setting "RS485 Transceiver Control" would activate 
  XON/XOFF, too
- Fixed bug: DL.LoadProgramOptions "" now displays the file open dialog correctly
- Fixed bug: Keyboard console in V1.8.10 introduces new timestamp after each 
  character
- Fixed bug: Keyboard console steals focus from other applications.
- Fixed bug: Run-time error 91 could occur when Docklight is closed with logging
  still running
- Fixed problem: Docklight goes to 100% CPU load with certain (virtual) COM port
  drivers.  

Docklight / Docklight Scripting V1.8.10 (03/2008)
New features and functions: 
- HTML log file format can be customized.
- New DL.GetChannelStatus() method to determine the communication channel status
  (closed, open, waiting for TCP connection, or error).
- New DL.PlaybackLogFile() method to replay a recorded communication. 
- New DL.SaveProgramOptions() / DL.LoadProgramOptions() methods to save and load
  Docklight user options (display settings, ...)
Fixes and Improvements:
- DL.OnSend_Peek() / DL.OnReceive_Peek(): optional parameter to return the value 
  in Docklight's ASC, HEX, Decimal or Binary representation
- DL.SetChannelSettings(): Additional "dontTest" argument to suppress the open/close
  attempts for testing purposes.
- Improved multiple screen support.
- Sequence editor recognizes HEX, Decimal or Binary data in various
  formats (e.g "5A A5 0F", "5A-A5-0F", "5A/A5/F" or "5AA50F")
- Sequence Editor mode (ASCII / HEX / Decimal ...) always follows the
  selected Communications Window Mode.
- Improved sequence editor behavior when deleting a single HEX character.
- Keyboard Console allows transmitting data by pasting from clipboard (up to 
  1034 characters).
- new menu Help->Contact E-Mail Support allows sending relevant user setttings, 
  to faciliate customer support.
- Baud rates down to 1 (one) can be selected now (Note: Make sure your COM
  port device can handle such low baud rates properly.) 
- Fixed compatibility problem with Keyspan USA-19H USB adapter
- Fixed bug: FileInput.OpenFile "" now uses file open dialog correctly
- Fixed bug: Docklight Scripting now sets working directory correctly when 
  started from command line.


Docklight / Docklight Scripting V1.7.37 (06/2007)
Fixes and Improvements:
- For ASCII log files, the ASCII character code 26 is now replaced by 127.
  ASCII code 26 serves as a end-of-file mark for text files and should be 
  avoided in text files.
- Improved behavior when CR only or LF only is used for end-of-line: 
  The line break is now inserted always before a comm.direction change, a new 
  date/time stamp or a comment.
- Improved behavior when a TCP client is immediately rejected: 2 seconds idle
  time before retry.
- Fixed bug: Class definitions with private member variables caused syntax
  error. 
- Fixed bug: Docklight V1.7 now stores COM > 16 correctly

Docklight / Docklight Scripting V1.7.33 (04/2007)
New features and functions: 
- Networking: Docklight Scripting can act as TCP client, TCP server or UDP peer
- new FileInput / FileOutput objects for easy and straightforward file I/O
- new DL.CalcChecksum() method for calculating CRCs of any type
- new DL.SetChannelSettings() / DL.GetChannelSettings() methods for changing
  the communication port and settings (baud rate, ...) while running a script.
- new DL.Quit() command to stop script execution
- new DL.GetDocklightTimeStamp() method to return a Docklight-style time stamp

Fixes and Improvements:
- COM1 to COM256 can be selected.
- Project Settings dialog shows available COM ports in dropdown list.
- Timing acurracy in monitoring mode improved.
- Improved transmit and receive buffering, less COM buffer overflows
  on high-speed connections.
- Monitoring mode additionally transmits the received data on the 
  opposite communication port (data forwarding).
- Communications display now buffers up to 128.000 characters.
- Log file buffers are now flushed after 1 seconds of inactivity.
- "RS485 Transceiver Control" now uses Windows RTS_CONTROL_TOGGLE mode with
  improved timing (Windows NT/2000/XP/Vista only)
- Parity Error Character: "(ignore)" for reading characters with wrong parity bit
- The "Edit Send/Receive Sequence" dialogs show the current cursor position 
  and support cursor keys (Page Up/Down, Cursor Up/Down, Home/End).
- Minimum time for "Send Sequence periodically" now 0.01 sec (before: 0.1 sec)
- Additional operations on sequence lists: Import from a project file, 
  swap receive and send sequence lists.
- Improved behavior for large sequence lists (> 100 sequences).
- Fixed rare crash in Sequence Editor after copying & pasting a sequence 
  using Ctrl+C and Ctrl+V.
- DL.UploadFile() supports raw binary data tranfer mode ("R").
- DL.UploadFile(), DL.StartLogging(), DL.OpenProject() show a file dialog 
  if an empty file path argument is passed.
- DL.StartLogging() closes a previously opened log file automatically, 
  instead of returning an error.
- DL.AddComment with additional formatting options.
- DL.ResetReceiveCounter now additionally resets the Receive Sequence detection
  algorithm, allowing easier resynch in complex protocols.
- Additional "!" function character arguments for temporarily changing the parity 
  settings within one Send Sequence. 
- Receive Sequence supports "&" Delay function character to detect pauses.
- DL.StartLogging uses current script directory by default (same behavior as
  DL.OpenProject or DL.UploadFile).
  
Docklight V1.6.23 / Docklight Scripting V1.6.23 (01/2007)
Fixes and Improvements:
- HTML help instead of Winhelp to prepare Docklight for Windows Vista
- Window size correctly remembered after closing Docklight, even when maximized.
- Font sizes < 10 now allowed (with additional warning) 
- Control characters except CR/LF can be completely suppressed in ASCII display
10:16 02.04.2007- Baud rates up to 9.999.999 Baud can be entered (NOTE: This does not mean
  Docklight can really process any kind of data at that speed.) 
- Fixed bug: A "CR only" produces an additional line break in HEX, Decimal and 
  Binary display
- Fixed bug: The keyboard console tool always sets the RTS line to high when 
  typing characters
- Fixed bug: The DCD line status is not displayed
- Fixed bug in Docklight Scripting: 
  UploadFile() does not send data if the script contains a DL_OnSend() procedure
- Fixed bug in Docklight Scripting: 
  RS485 Tranceiver Control does not reset RTS while a script is executed
- Fixed bug in Docklight Scripting: 
  BREAK state introduces additional spaces in HEX, Decimal and Binary display

Docklight V1.6.8 / Docklight Scripting V1.6.8 (04/2005)
Fixes and Improvements:
- Flow control setting "RS485 Transceiver Control" is now correctly applied for
  the Keyboard Console tool.
- F12 key brings up the notepad window, even if minimized

Docklight V1.6.7 / Docklight Scripting V1.6.7 (03/2005)
New features and functions: 
- "Communication Filter" to hide the original serial data of one or both 
  communication channels
- Notepad (F12 key) for project documentation
- "Drag and Drop" support for project files and scripts
- Docklight Scripting: new method "UploadFile" for file transfer
- Docklight Scripting: new special function characters in Send or 
  Receive Sequences (set/reset handshake signals, add delay between characters, 
  send or detect a "break" state)
Fixes and Improvements:
- Keyboard Shortcuts for ASCII code > 126
- DEL key for Send/Receive Sequence lists
- Docklight Scripting: Increased maximum script size (510KB instead of 64KB)
- Inreased Docklight send/receive queue size to allow character-by-character 
  processing using DL_OnReceive()
- Extended documentation and sample scripts to demonstrate Docklight Scripting's 
  data analysis and manipulation capabilities.
- Fixed bug: Docklight crashes when a USB-to-RS232 device is removed while 
  communication is running. 
- Fixed problem: When using Option "Flow Control: RS485 transceiver control",
  the RTS line is reset too early, especially when the standard 16 byte fifo 
  transmit buffer of the UART is enabled.
- Fixed bug: Missing characters in ASCII window / Formatted Text Output mode,
  when characters are received one-by-one and a single <CR> is used for end-of-line

Docklight Scripting V1.5.2 (09/2004)
New features and functions:
- Extended syntax for the DL.SendSequence command to allow hex, decimal or binary
  Send Sequence parameters or sending custom data sequences.
- DL_OnSend() function to support automatic checksum calculations
- DL_OnReceive() function to support automatic evaluation of the received data,
  including analysing the received wildcard data
Fixes:
- Fixed bug: The DL.OpenProject command did not close the communication

Docklight V1.4.14 / Docklight Scripting V1.4.14  (06/2004)
Fixes:
- Fixed bug: Missing line breaks on Windows 98
- Fixed bug: "Clear Communication Window" does not delete the entire contents of
  all four commmunications window representations (ASCII/HEX/Decimal/Binary)

Docklight V1.4.12 / Docklight Scripting V1.4.12  (05/2004)
New features and functions:
- New Docklight edition "Scripting" with built-in script editor for automated testing.
  Available as an upgrade for Docklight standard users. 
Fixes & Improvements:
- Non-standard baud rates can be used: Type any integer number between 110 and 999999
  in the corresponding dialog box. If the chosen rate is actually applied, depends on 
  the serial UART you are using. Non-standard baud rates may not work correctly when 
  "Flow Control" options are used.
- New Mode "Flow Control: Hardware Handshaking - RTS/CTS (Single Byte Mode):
  A sequence is placed byte-by-byte into the transmit buffer and CTS is checked before 
  each new character.
- New mode "Flow Control: RTS high while sending". This is to support RS485
  converters and related applications where the transceiver requires the PCs RTS signal
  to enable/disable the transmission.
- Improved communication processing and timing accuracy, which makes the "TweakComm" 
  utility provided to some users obsolete.
- ASCII Communication Window: Uniform behavior for different end-of-line standards:
  CR only, CR+LF, LF+CR

V1.3.38 (01/2004)
Fixes:
- Fixed bug: receive sequence detection ignores incoming sequence, 
  if immediately before the sequence a few characters have already been 
  considered as a part of this sequence.   
  See http://www.docklight.de/troubleshooting_en.htm for details.
- Fixed bug: edit sequence dialog crashes when marking a sequence, then 
  moving to the empty sequence at the end of the list (Index ">" button) and
  pasting into the empty sequence.
- Fixed bug: crash when trying to paste a very large document (> 32K) from clipboard.

V1.3 (11/2003)
New features and functions:
- Powerful clipboard support within the sequence lists and the sequence 
  editor: cut, copy & paste entire sequences or parts of it. Copy & paste 
  sequence data from external applications (MS Word, Notepad).
- Wildcard support for send sequences, receive sequences and find function
- New receive sequence actions: insert time stamp, stop communication
- New time stamp options: pause detection, e.g. for RS485 monitoring.
- Keyboard console: keyboard input is directly sent to the COM port.
- New Project settings: optional flow control support, 
  manual RTS/DTR control, hardware handshaking, 
  software handshaking (XON/XOFF).
Fixes & Improvements:
- Fixed bug when printing on HP Laserjet 5M
- Improved sequence list management (reordering, keyboard support)
- Improved find function (ignores date/time stamps and additional comments
  in HEX, decimal and binary mode) 
   
V1.2 (02/2003)
New features and functions:
- Improved performance, especially when monitoring transmissions 
  with a high amount of data
- Time stamps with 1/100s precision
- New display options to further increase processing speed
- New snapshot function to catch a very rare sequence within the protocol data
  plus the preceding and trailing communication
- New demo project and additional documentation for getting started
Fixes:
- Now works on any Windows language edition, especially Asian editions or 
  others using DBCS (double byte character set) 
- Fixed bug when using sequence names which include a comma (",")

V1.1 (09/2002)
New features and functions:
- Creating log files in HTML format
- Communication settings are now stored as project data
- Improved parity error handling
- Disabling the communication window while logging possible (e.g. while 
  monitoring high-speed communication) 
- Revised and streamlined user interface
Fixes:
- 4800 Baud now available in the communication settings dialog

V1.0 (04/2002)
First release