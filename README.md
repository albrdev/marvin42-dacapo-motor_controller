To build this in PlatformIO you need to use "git clone --recurse-submodules"

## BlueSMiRF setup (slave)
| Command          | Description                                          |
| ---------------- | ---------------------------------------------------- |
| GB               | Get the device's MAC address (needed for master)     |
| SN,<em>name</em> | Set name (optional)                                  |
| SP,<em>pin</em>  | Set PIN code                                         |
| SU,115200,N      | Set baud rate and disable parity (must match master) |
| SL,N             | Only disable parity (must match master)              |
| SM,0             | Set mode to <em>slave</em>                           |
| SA,4             | Set authentication mode to <em>PIN code</em>         |
| R,1              | Reboot the device                                    |
