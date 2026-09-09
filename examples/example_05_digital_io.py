"""Write a digital output, and read the robot's inputs.

WHICH physical signal you get depends on the RSI context, not on this script.
With RSI_EthernetConfig.xml and KUKA's matching RSI_Ethernet.rsi:

    MAP2DIGOUT1  Index=20, DataSize=Word  ->  $OUT[161] .. $OUT[176]
    DIGIN1       Index=1,  DataSize=Byte  ->  $IN[9]   .. $IN[16]

With byte/word addressing, `Index` counts BYTES, so byte 20 starts at
$OUT[20 * 8 + 1] = $OUT[161]. That is also why the channel numbers below are
BIT POSITIONS in the word, not $OUT/$IN numbers.

To drive a specific output instead - a gripper on $OUT[5], say - give a
MAP2DIGOUT object Index=5 and DataSize=Bit in RSIVisual. Then the channel
number is the real output number. See docs/rsi-objects.md, section 4.
"""
import sys

from RSIPI import RSIAPI

from _confirm import confirm

if __name__ == '__main__':
    from multiprocessing import freeze_support
    freeze_support()

    # Override from the command line to run against the echo server:

    #   python examples/dry_run.py examples/example_05_digital_io.py

    CONFIG = sys.argv[1] if len(sys.argv) > 1 else 'RSI_EthernetConfig.xml'

    api = RSIAPI(CONFIG)
    api.start()

    # Bit 0 of the DiO word -> $OUT[161] with the context above.
    # Each call is a read-modify-write of the whole word, so several outputs
    # can be changed together and land in the same 4 ms cycle.
    if confirm("Switch on $OUT[161] and $OUT[162]",
               "Bits 0 and 1 of the DiO word. Make sure nothing is wired to them."):
        api.io.set_output(1, True)     # DiO = 0b00000001 = 1
        api.io.set_output(2, True)     # DiO = 0b00000011 = 3  -> $OUT[161], [162]

    # Reading inputs: bit 0 of the DiL word, i.e. $IN[9] here.
    print("input 1 (=$IN[9]):", api.io.get_input(1))

    # Digout.o1-3 in the SEND section is the robot's read-back of $OUT[1..3].
    # It is read-only - outputs are commanded through the RECEIVE side.
    print("robot reports Digout:", api.client.send_variables.get("Digout"))

    # Switch them off regardless - harmless if they were never switched on,
    # and it leaves the cell as we found it.
    api.io.set_output(1, False)
    api.io.set_output(2, False)
    api.stop()
