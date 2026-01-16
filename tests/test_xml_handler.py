"""Tests for XMLGenerator."""
import pytest
import xml.etree.ElementTree as ET
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.xml_handler import XMLGenerator


class TestGenerateSendXML:
    """Tests for XMLGenerator.generate_send_xml()"""

    def test_simple_values(self):
        """Test XML generation with simple scalar values."""
        send_vars = {
            "IPOC": 12345,
            "DiL": 1
        }
        network_settings = {"sentype": "ImFree"}

        xml_str = XMLGenerator.generate_send_xml(send_vars, network_settings)

        # Parse and verify
        root = ET.fromstring(xml_str)
        assert root.tag == "Sen"
        assert root.get("Type") == "ImFree"
        assert root.find("IPOC").text == "12345"
        assert root.find("DiL").text == "1"

    def test_nested_dict_values(self):
        """Test XML generation with nested dictionary values (attributes)."""
        send_vars = {
            "RKorr": {"X": 100.5, "Y": -50.25, "Z": 0.0}
        }
        network_settings = {"sentype": "TestType"}

        xml_str = XMLGenerator.generate_send_xml(send_vars, network_settings)

        root = ET.fromstring(xml_str)
        rkorr = root.find("RKorr")
        assert rkorr is not None
        assert float(rkorr.get("X")) == 100.50
        assert float(rkorr.get("Y")) == -50.25
        assert float(rkorr.get("Z")) == 0.0

    def test_free_field_skipped(self):
        """Test that FREE field is skipped in XML output."""
        send_vars = {
            "IPOC": 1,
            "FREE": 999
        }
        network_settings = {"sentype": "ImFree"}

        xml_str = XMLGenerator.generate_send_xml(send_vars, network_settings)

        root = ET.fromstring(xml_str)
        assert root.find("FREE") is None
        assert root.find("IPOC") is not None

    def test_mixed_values(self):
        """Test XML generation with mixed scalar and nested values."""
        send_vars = {
            "IPOC": 5000,
            "RKorr": {"X": 10.0, "Y": 20.0},
            "Digout": {"o1": 1, "o2": 0}
        }
        network_settings = {"sentype": "Mixed"}

        xml_str = XMLGenerator.generate_send_xml(send_vars, network_settings)

        root = ET.fromstring(xml_str)
        assert root.find("IPOC").text == "5000"
        assert float(root.find("RKorr").get("X")) == 10.0
        assert float(root.find("Digout").get("o1")) == 1.0


class TestGenerateReceiveXML:
    """Tests for XMLGenerator.generate_receive_xml()"""

    def test_simple_receive(self):
        """Test receive XML generation with scalar values."""
        receive_vars = {
            "IPOC": 12345,
            "BMode": "Active"
        }

        xml_str = XMLGenerator.generate_receive_xml(receive_vars)

        root = ET.fromstring(xml_str)
        assert root.tag == "Rob"
        assert root.get("Type") == "KUKA"
        assert root.find("IPOC").text == "12345"
        assert root.find("BMode").text == "Active"

    def test_nested_receive(self):
        """Test receive XML generation with position data."""
        receive_vars = {
            "RIst": {"X": 500.0, "Y": 100.0, "Z": 800.0},
            "ASPos": {"A1": 0.0, "A2": -45.0, "A3": 90.0}
        }

        xml_str = XMLGenerator.generate_receive_xml(receive_vars)

        root = ET.fromstring(xml_str)
        rist = root.find("RIst")
        assert float(rist.get("X")) == 500.0
        assert float(rist.get("Z")) == 800.0

        aspos = root.find("ASPos")
        assert float(aspos.get("A2")) == -45.0
