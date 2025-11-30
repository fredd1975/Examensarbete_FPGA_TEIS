-- ==========================================================================
-- Company        :  TEIS AB
-- Engineer       :  Freddy Avaria
-- Project        :  FPGA & TEIS Thesis - PXE-Based Remote Reboot System
-- Create Date    :  2025 Oct.27
-- Design name    :  simple_byte_rom.vhd
-- Target devices :  ALTERA MAX 10
-- Tool versions  :  Quartus v.18.1 and ModelSim
--
-- Description: Simple ROM file acting as storage  (VHDL-93)
--              Byte-wide ROM with 1-cycle synchronous read
--              Ports: clk, addr(31:0), rd, q(7:0)
--              Depth fixed at 4096 bytes (adjust inside if needed)
-- ==========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity simple_byte_rom is
   port (
      clk  : in  std_logic;                   -- clock
      addr : in  unsigned(31 downto 0);       -- ROM address
      rd   : in  std_logic;                   -- read enable
      q    : out std_logic_vector(7 downto 0) -- ROM data out
      );
end entity simple_byte_rom;

architecture rtl of simple_byte_rom is
   constant ROM_SIZE : natural := 4096;  -- bytes
   subtype byte_t is std_logic_vector(7 downto 0);
   type mem_t is array (0 to ROM_SIZE-1) of byte_t;

  -- Initialize to zeros; replace with a file-based init if desired
   signal mem : mem_t := (others => (others => '0'));

   signal addr_r : unsigned(31 downto 0) := (others => '0');
begin

  -- 1-cycle synchronous read:
  --   on rd='1', capture address; q updates next cycle
   process(clk)
   begin
      if rising_edge(clk) then
         if rd = '1' then
            addr_r <= addr;
         end if;

      -- Use lower address bits to index 4 KiB
         q <= mem(to_integer(addr_r(11 downto 0)));
      end if;
   end process;

end architecture rtl;
