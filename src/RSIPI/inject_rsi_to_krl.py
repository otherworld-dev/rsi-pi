def inject_rsi_to_krl(input_file, output_file=None, rsi_config="RSIGatewayv1.rsi"):
    """
    Injects RSI commands into a KUKA KRL (.src) program file by:
    - Declaring RSI variables.
    - Creating the RSI context with a given configuration.
    - Starting and stopping RSI execution around the program body.

    Args:
        input_file (str): Path to the original KRL file.
        output_file (str, optional): Output file to save modified code. Defaults to overwriting input_file.
        rsi_config (str): Name of the RSI configuration (usually ending in .rsi).
    """
    if output_file is None:
        output_file = input_file  # Overwrite original file if no output specified

    # RSI declarations to insert at top
    rsi_start = """
; RSI Variable Declarations
DECL INT ret
DECL INT CONTID
"""

    # RSI context creation and startup block
    rsi_middle = f"""
; Create RSI Context
ret = RSI_CREATE("{rsi_config}", CONTID, TRUE)
IF (ret <> RSIOK) THEN
    HALT
ENDIF

; Start RSI Execution
ret = RSI_ON(#RELATIVE)
IF (ret <> RSIOK) THEN
    HALT
ENDIF
"""

    # RSI shutdown block to insert before END
    rsi_end = """
; Stop RSI Execution
ret = RSI_OFF()
IF (ret <> RSIOK) THEN
    HALT
ENDIF
"""

    # Read original KRL file into memory
    with open(input_file, "r") as file:
        lines = file.readlines()

    # Identify key structural markers in the KRL program
    header_end, ini_end, end_start = None, None, None

    for i, line in enumerate(lines):
        if line.strip().startswith("DEF"):
            header_end = i
        elif line.strip().startswith(";ENDFOLD (INI)"):
            ini_end = i
        elif line.strip().startswith("END"):
            end_start = i

    # Validate presence of required sections
    if header_end is None or ini_end is None or end_start is None:
        raise ValueError("Required markers (DEF, ;ENDFOLD (INI), END) not found in KRL file.")

    # Inject modified contents into new or overwritten file
    with open(output_file, "w") as file:
        file.writelines(lines[:header_end + 1])  # Preserve header
        file.write(rsi_start)                   # Add RSI declarations
        file.writelines(lines[header_end + 1:ini_end + 1])  # Preserve INI block
        file.write(rsi_middle)                  # Insert RSI start commands
        file.writelines(lines[ini_end + 1:end_start])  # Preserve main body
        file.write(rsi_end)                     # Insert RSI stop commands
        file.write(lines[end_start])            # Write final END line


# Example usage
if __name__ == "__main__":
    inject_rsi_to_krl("my_program.src", "my_program_rsi.src")
