import csv
import logging
import re
from collections import OrderedDict

class KRLParser:
    """
    Parses KUKA KRL .src and .dat files to extract TCP setpoints
    and exports them into a structured CSV format.
    """

    def __init__(self, src_file, dat_file):
        self.src_file = src_file
        self.dat_file = dat_file
        self.positions = OrderedDict()  # Maintain order of appearance
        self.labels_to_extract = []     # Store labels found in .src (e.g., XP310, XP311)

    def parse_src(self):
        """
        Parses the .src file to extract motion commands and their labels (e.g., PTP XP310).
        """
        move_pattern = re.compile(r"\bPTP\s+(\w+)", re.IGNORECASE)

        with open(self.src_file, 'r', encoding='utf-8') as file:
            for line in file:
                match = move_pattern.search(line)
                if match:
                    label = match.group(1).strip().upper()
                    if label not in self.labels_to_extract:
                        self.labels_to_extract.append(label)

    def parse_dat(self):
        """
        Parses the .dat file and retrieves Cartesian coordinates for each label.
        """
        pos_pattern = re.compile(r"DECL\s+E6POS\s+(\w+)\s*=\s*\{([^}]*)\}", re.IGNORECASE)

        with open(self.dat_file, 'r', encoding='utf-8') as file:
            for line in file:
                match = pos_pattern.search(line)
                if match:
                    label = match.group(1).strip().upper()
                    coords_text = match.group(2)

                    coords = {}
                    for entry in coords_text.split(','):
                        key_value = entry.strip().split()
                        if len(key_value) == 2:
                            key, value = key_value
                            try:
                                if key in ["S", "T"]:
                                    coords[key] = int(float(value))
                                else:
                                    coords[key] = float(value)
                            except ValueError:
                                coords[key] = 0  # fallback

                    self.positions[label] = coords

    def export_csv(self, output_file):
        """
        Writes the extracted Cartesian positions into a structured CSV file,
        skipping any deleted/missing points.
        """
        fieldnames = ["Sequence", "PosRef", "X", "Y", "Z", "A", "B", "C", "S", "T"]

        with open(output_file, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            sequence_number = 0  # Only count real points

            for label in self.labels_to_extract:
                coords = self.positions.get(label)
                if coords:
                    writer.writerow({
                        "Sequence": sequence_number,
                        "PosRef": label,
                        "X": coords.get("X", 0),
                        "Y": coords.get("Y", 0),
                        "Z": coords.get("Z", 0),
                        "A": coords.get("A", 0),
                        "B": coords.get("B", 0),
                        "C": coords.get("C", 0),
                        "S": coords.get("S", 0),
                        "T": coords.get("T", 0),
                    })
                    sequence_number += 1
                else:
                    logging.warning(f"Skipped missing/deleted point: {label}")

        logging.info(f"CSV exported successfully to {output_file} with {sequence_number} points.")


# Optional CLI usage
if __name__ == "__main__":
    parser = KRLParser("path/to/file.src", "path/to/file.dat")
    parser.parse_src()
    parser.parse_dat()
    parser.export_csv("path/to/output.csv")
