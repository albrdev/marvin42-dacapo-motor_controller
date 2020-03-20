To build this in PlatformIO you need to use "git clone --recurse-submodules"

BlueSMiRF setup (slave):
SN,name         # Set name
SP,pin          # Set PIN
SM,0            # Set mode to slave
SU,115200,N     # Optional: Set baud rate and parity
SL,N            # Optional: Set only parity
SA,4            # Set authentication mode to PIN code
R,1             # Reboot
