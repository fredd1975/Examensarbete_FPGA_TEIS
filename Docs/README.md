# Examensarbete_FPGA_TEIS
## Thesis work by Freddy A.

This thesis recreates the basic steps of a PXE-style network boot inside an FPGA and adds a hardware accelerator that can observe and analyze the process in real time. A simple TFTP client and server were built in VHDL and connected internally so they can exchange boot messages without using any external network hardware. The accelerator watches this traffic as it flows through the system and identifies the key parts of each message, making it easier to understand how the protocol works and how it could be sped up in hardware. The design was tested both in simulation and on the FPGA board, resulting in a clear learning platform for network booting and for developing future FPGA-based acceleration techniques.

