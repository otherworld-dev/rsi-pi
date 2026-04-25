"""Tests for ConfigParser."""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from RSIPI.config_parser import ConfigParser

# Path to the real config file in the project root
CONFIG_FILE = os.path.join(os.path.dirname(__file__), '..', 'RSI_EthernetConfig.xml')


@pytest.fixture
def parser():
    """Create a ConfigParser from the real config file."""
    return ConfigParser(CONFIG_FILE)


class TestParseWithoutError:
    """Verify that the config file parses successfully."""

    def test_parser_initializes(self, parser):
        """ConfigParser should initialize without raising."""
        assert parser is not None

    def test_send_variables_is_dict(self, parser):
        """send_variables should be a dict after parsing."""
        assert isinstance(parser.send_variables, dict)

    def test_receive_variables_is_dict(self, parser):
        """receive_variables should be a dict after parsing."""
        assert isinstance(parser.receive_variables, dict)


class TestNetworkSettings:
    """Verify network_settings extraction from <CONFIG>."""

    def test_has_ip(self, parser):
        """network_settings must contain 'ip'."""
        assert "ip" in parser.network_settings
        assert parser.network_settings["ip"] == "10.10.10.10"

    def test_has_port(self, parser):
        """network_settings must contain 'port' as int."""
        assert "port" in parser.network_settings
        assert parser.network_settings["port"] == 64000
        assert isinstance(parser.network_settings["port"], int)

    def test_has_sentype(self, parser):
        """network_settings must contain 'sentype'."""
        assert "sentype" in parser.network_settings
        assert parser.network_settings["sentype"] == "ImFree"

    def test_has_onlysend(self, parser):
        """network_settings must contain 'onlysend' as bool."""
        assert "onlysend" in parser.network_settings
        assert parser.network_settings["onlysend"] is False

    def test_get_network_settings_matches(self, parser):
        """get_network_settings() should return the same dict."""
        assert parser.get_network_settings() == parser.network_settings


class TestSendVariables:
    """Verify send_variables contains expected keys from the config file."""

    def test_contains_rist(self, parser):
        """RIst (robot actual position) should be in send_variables."""
        assert "RIst" in parser.send_variables

    def test_rist_has_cartesian_keys(self, parser):
        """RIst should expand to X, Y, Z, A, B, C sub-keys."""
        rist = parser.send_variables["RIst"]
        assert isinstance(rist, dict)
        for axis in ["X", "Y", "Z", "A", "B", "C"]:
            assert axis in rist

    def test_contains_rsol(self, parser):
        """RSol (robot commanded position) should be in send_variables."""
        assert "RSol" in parser.send_variables

    def test_contains_ipoc(self, parser):
        """IPOC timestamp should always be in send_variables."""
        assert "IPOC" in parser.send_variables

    def test_contains_delay(self, parser):
        """Delay should be in send_variables."""
        assert "Delay" in parser.send_variables

    def test_contains_digout(self, parser):
        """Digout group should be parsed from dotted tags."""
        assert "Digout" in parser.send_variables

    def test_digout_has_channels(self, parser):
        """Digout should contain o1, o2, o3 sub-keys from the config."""
        digout = parser.send_variables["Digout"]
        assert isinstance(digout, dict)
        for ch in ["o1", "o2", "o3"]:
            assert ch in digout

    def test_contains_source1(self, parser):
        """Source1 should be in send_variables as a DOUBLE default."""
        assert "Source1" in parser.send_variables
        assert parser.send_variables["Source1"] == 0.0

    def test_contains_dil(self, parser):
        """DiL should be in send_variables as a LONG default."""
        assert "DiL" in parser.send_variables
        assert parser.send_variables["DiL"] == 0


class TestReceiveVariables:
    """Verify receive_variables contains expected keys."""

    def test_contains_rkorr(self, parser):
        """RKorr (correction values) should be in receive_variables."""
        assert "RKorr" in parser.receive_variables

    def test_rkorr_has_cartesian_keys(self, parser):
        """RKorr should have X, Y, Z, A, B, C from individual tags."""
        rkorr = parser.receive_variables["RKorr"]
        assert isinstance(rkorr, dict)
        for axis in ["X", "Y", "Z", "A", "B", "C"]:
            assert axis in rkorr

    def test_contains_estr(self, parser):
        """EStr should be in receive_variables."""
        assert "EStr" in parser.receive_variables

    def test_contains_free(self, parser):
        """FREE should be in receive_variables."""
        assert "FREE" in parser.receive_variables

    def test_contains_dio(self, parser):
        """DiO should be in receive_variables."""
        assert "DiO" in parser.receive_variables


class TestDefPrefixStripping:
    """Verify that DEF_ prefix is stripped from tags."""

    def test_rist_not_def_rist(self, parser):
        """Key should be 'RIst', not 'DEF_RIst'."""
        assert "DEF_RIst" not in parser.send_variables
        assert "RIst" in parser.send_variables

    def test_rsol_not_def_rsol(self, parser):
        """Key should be 'RSol', not 'DEF_RSol'."""
        assert "DEF_RSol" not in parser.send_variables
        assert "RSol" in parser.send_variables

    def test_estr_not_def_estr(self, parser):
        """Key should be 'EStr', not 'DEF_EStr'."""
        assert "DEF_EStr" not in parser.receive_variables
        assert "EStr" in parser.receive_variables

    def test_tech_not_def_tech(self, parser):
        """Tech keys should not retain DEF_ prefix."""
        for key in parser.send_variables:
            assert not key.startswith("DEF_")
        for key in parser.receive_variables:
            assert not key.startswith("DEF_")


class TestIPOCAlwaysPresent:
    """IPOC must always be in send_variables, even if not in config."""

    def test_ipoc_present(self, parser):
        """IPOC should be in send_variables."""
        assert "IPOC" in parser.send_variables

    def test_ipoc_forced_when_missing(self, tmp_path):
        """IPOC should be injected even if the config has no DEF_IPOC."""
        # Create a minimal config with no IPOC element
        minimal_xml = tmp_path / "minimal.xml"
        minimal_xml.write_text("""<ROOT>
            <CONFIG>
                <IP_NUMBER>127.0.0.1</IP_NUMBER>
                <PORT>12345</PORT>
                <SENTYPE>Test</SENTYPE>
                <ONLYSEND>FALSE</ONLYSEND>
            </CONFIG>
            <SEND><ELEMENTS>
                <ELEMENT TAG="DEF_RIst" TYPE="DOUBLE" INDX="INTERNAL" />
            </ELEMENTS></SEND>
            <RECEIVE><ELEMENTS>
                <ELEMENT TAG="RKorr.X" TYPE="DOUBLE" INDX="1" />
            </ELEMENTS></RECEIVE>
        </ROOT>""")
        p = ConfigParser(str(minimal_xml))
        assert "IPOC" in p.send_variables


class TestTechKeyMerging:
    """Verify that Tech.CX and Tech.TX keys merge into a single 'Tech' dict."""

    def test_tech_merged_in_send(self, parser):
        """Tech.C1 in send config should merge into a single 'Tech' key."""
        assert "Tech" in parser.send_variables
        # Should not have the dotted form
        for key in parser.send_variables:
            assert not key.startswith("Tech.")

    def test_tech_merged_in_receive(self, parser):
        """Tech.T2 in receive config should merge into a single 'Tech' key."""
        assert "Tech" in parser.receive_variables
        for key in parser.receive_variables:
            assert not key.startswith("Tech.")

    def test_tech_contains_sub_keys(self, parser):
        """Merged Tech dict should contain the sub-keys from the internal structure."""
        tech_send = parser.send_variables["Tech"]
        assert isinstance(tech_send, dict)
        # Tech.C1 should have expanded to C11, C12, ... C110
        assert "C11" in tech_send

    def test_tech_receive_contains_sub_keys(self, parser):
        """Tech in receive should have T2x keys from Tech.T2."""
        tech_recv = parser.receive_variables["Tech"]
        assert isinstance(tech_recv, dict)
        assert "T21" in tech_recv


class TestGetDefaultValue:
    """Test the static get_default_value helper."""

    def test_bool_default(self):
        assert ConfigParser.get_default_value("BOOL") is False

    def test_string_default(self):
        assert ConfigParser.get_default_value("STRING") == ""

    def test_long_default(self):
        assert ConfigParser.get_default_value("LONG") == 0

    def test_double_default(self):
        assert ConfigParser.get_default_value("DOUBLE") == 0.0

    def test_unknown_default(self):
        assert ConfigParser.get_default_value("UNKNOWN") is None


class TestRenameTechKeys:
    """Test the static rename_tech_keys helper."""

    def test_merges_tech_keys(self):
        d = {"Tech.C1": {"C11": 0}, "Tech.T2": {"T21": 0}, "Other": 5}
        ConfigParser.rename_tech_keys(d)
        assert "Tech" in d
        assert "Tech.C1" not in d
        assert "Tech.T2" not in d
        assert "C11" in d["Tech"]
        assert "T21" in d["Tech"]
        assert d["Other"] == 5

    def test_no_tech_keys_leaves_dict_unchanged(self):
        d = {"RIst": {"X": 0}, "IPOC": 0}
        ConfigParser.rename_tech_keys(d)
        assert "Tech" not in d
        assert "RIst" in d
