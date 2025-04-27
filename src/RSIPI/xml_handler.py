import xml.etree.ElementTree as ET

class XMLGenerator:
    """
    Converts structured dictionaries of RSI send/receive variables into
    valid XML strings for UDP transmission to/from the robot controller.
    """

    @staticmethod
    def generate_send_xml(send_variables, network_settings):
        """
        Build an outgoing XML message based on the current send variables.

        Args:
            send_variables (dict): Structured dictionary of values to send.
            network_settings (dict): Contains 'sentype' used for the root element.

        Returns:
            str: XML-formatted string ready for UDP transmission.
        """
        root = ET.Element("Sen", Type=network_settings["sentype"])

        # Convert structured keys (e.g. RKorr, Tech) and flat elements (e.g. IPOC)
        for key, value in send_variables.items():
            if key == "FREE":
                continue  # Skip unused placeholder fields

            if isinstance(value, dict):
                element = ET.SubElement(root, key)
                for sub_key, sub_value in value.items():
                    element.set(sub_key, f"{float(sub_value):.2f}")
            else:
                ET.SubElement(root, key).text = str(value)

        return ET.tostring(root, encoding="utf-8").decode()

    @staticmethod
    def generate_receive_xml(receive_variables):
        """
        Build an incoming XML message for emulation/testing purposes.

        Args:
            receive_variables (dict): Structured dictionary of values to simulate reception.

        Returns:
            str: XML-formatted string mimicking a KUKA robot's reply.
        """
        root = ET.Element("Rob", Type="KUKA")

        for key, value in receive_variables.items():
            if isinstance(value, dict) or hasattr(value, "items"):
                element = ET.SubElement(root, key)
                for sub_key, sub_value in value.items():
                    element.set(sub_key, f"{float(sub_value):.2f}")
            else:
                ET.SubElement(root, key).text = str(value)

        return ET.tostring(root, encoding="utf-8").decode()
