"""Tests for generating an Ethernet config from a context, and for deploy.

The strongest check available: regenerate the config for every shipped
context from its own .rsi.xml and require it to reproduce the hand-authored
file channel for channel. Those six configs were written by hand and verified
against a real controller, so matching them is real evidence rather than the
generator agreeing with itself.
"""
import xml.etree.ElementTree as ET

import pytest

from RSIPI import available_contexts, context, context_files
from RSIPI.config_builder import build_config
from RSIPI.deploy import deploy
from RSIPI.exceptions import RSIConfigError


def _channels(source, is_text=False):
    root = ET.fromstring(source) if is_text else ET.parse(source).getroot()
    out = {}
    for section in ("SEND", "RECEIVE"):
        for element in root.find(f"{section}/ELEMENTS"):
            index = element.get("INDX")
            if index.isdigit():
                out[(section, int(index))] = (element.get("TAG"),
                                              element.get("TYPE"))
    return out


def _context_xml(name):
    return str(next(f for f in context_files(name)
                    if f.name.endswith(".rsi.xml")))


class TestReproducesShippedConfigs:
    @pytest.mark.parametrize("name", available_contexts())
    def test_generated_config_matches_the_shipped_one(self, name):
        shipped_path = context(name)
        settings = ET.parse(shipped_path).getroot().find("CONFIG")
        generated = build_config(
            _context_xml(name),
            settings.find("IP_NUMBER").text.strip(),
            int(settings.find("PORT").text.strip()),
            sentype=settings.find("SENTYPE").text.strip(),
            onlysend=settings.find("ONLYSEND").text.strip().upper() == "TRUE",
            internals=["DEF_MACur"] if name == "max" else None,
        )
        assert _channels(generated, is_text=True) == _channels(shipped_path)

    @pytest.mark.parametrize("name", available_contexts())
    def test_generated_config_is_wellformed_and_parses(self, name):
        settings = ET.parse(context(name)).getroot().find("CONFIG")
        xml = build_config(_context_xml(name), "10.10.10.10", 64000)
        root = ET.fromstring(xml)          # raises if malformed
        assert root.find("CONFIG/IP_NUMBER").text == "10.10.10.10"
        assert root.find("CONFIG/PORT").text == "64000"


class TestNetworkSettings:
    def test_onlysend_flag_is_written(self):
        xml = build_config(_context_xml("basic"), "1.2.3.4", 1234, onlysend=True)
        assert ET.fromstring(xml).find("CONFIG/ONLYSEND").text == "TRUE"
        xml = build_config(_context_xml("basic"), "1.2.3.4", 1234, onlysend=False)
        assert ET.fromstring(xml).find("CONFIG/ONLYSEND").text == "FALSE"

    def test_sentype_is_written(self):
        xml = build_config(_context_xml("basic"), "1.2.3.4", 1234, sentype="Test")
        assert ET.fromstring(xml).find("CONFIG/SENTYPE").text == "Test"

    def test_extra_internal_tags_are_added(self):
        xml = build_config(_context_xml("basic"), "1.2.3.4", 1234,
                           internals=["DEF_MACur"])
        tags = [e.get("TAG") for e in ET.fromstring(xml).find("SEND/ELEMENTS")]
        assert "DEF_MACur" in tags


class TestDerivation:
    """The details that are easy to get wrong by hand."""

    def test_joint_feedback_only_when_the_context_corrects_joints(self):
        joints = [e.get("TAG") for e in
                  ET.fromstring(build_config(_context_xml("joints"), "1.2.3.4", 1)
                                ).find("SEND/ELEMENTS")]
        basic = [e.get("TAG") for e in
                 ET.fromstring(build_config(_context_xml("basic"), "1.2.3.4", 1)
                               ).find("SEND/ELEMENTS")]
        assert "DEF_AIPos" in joints
        assert "DEF_AIPos" not in basic

    def test_external_axis_inputs_are_not_collapsed(self):
        """AXISCORREXT's inputs start at 7, so indexing by raw InIdx would run
        off the tag list and label every channel EKorr.E1."""
        xml = build_config(_context_xml("full"), "1.2.3.4", 1)
        tags = {e.get("TAG") for e in ET.fromstring(xml).find("RECEIVE/ELEMENTS")}
        assert {f"EKorr.E{i}" for i in range(1, 7)} <= tags

    def test_a_correction_wins_a_shared_channel(self):
        """In Full, MAP2SEN_PREA reads the same channels as POSCORR."""
        xml = build_config(_context_xml("full"), "1.2.3.4", 1)
        by_channel = {int(e.get("INDX")): e.get("TAG")
                      for e in ET.fromstring(xml).find("RECEIVE/ELEMENTS")
                      if e.get("INDX").isdigit()}
        assert by_channel[1] == "RKorr.X"

    def test_spare_channels_are_declared(self):
        xml = build_config(_context_xml("basic"), "1.2.3.4", 1)
        by_channel = {int(e.get("INDX")): e.get("TAG")
                      for e in ET.fromstring(xml).find("RECEIVE/ELEMENTS")
                      if e.get("INDX").isdigit()}
        assert by_channel[7] == "FREE"

    def test_word_sized_digout_becomes_a_readback_word(self):
        xml = build_config(_context_xml("joints"), "1.2.3.4", 1)
        tags = {e.get("TAG") for e in ET.fromstring(xml).find("SEND/ELEMENTS")}
        assert "DoutW" in tags


class TestFailures:
    def test_context_without_ethernet_is_rejected(self, tmp_path):
        path = tmp_path / "NoEth.rsi.xml"
        path.write_text('<RSIDATA><RSIObject ObjType="POSCORR" ObjTypeID="27" '
                        'ObjID="POSCORR1" /></RSIDATA>', encoding="utf-8")
        with pytest.raises(RSIConfigError, match="ETHERNET"):
            build_config(str(path), "1.2.3.4", 1)

    def test_unknown_object_is_reported_not_guessed(self, tmp_path):
        path = tmp_path / "Odd.rsi.xml"
        path.write_text(
            '<RSIDATA>'
            '<RSIObject ObjType="WIDGET" ObjTypeID="999" ObjID="WIDGET1" />'
            '<RSIObject ObjType="ETHERNET" ObjTypeID="64" ObjID="ETHERNET1">'
            '<Inputs><Input InIdx="1" OutObjID="WIDGET1" OutIdx="1" /></Inputs>'
            '</RSIObject></RSIDATA>', encoding="utf-8")
        with pytest.raises(RSIConfigError, match="WIDGET"):
            build_config(str(path), "1.2.3.4", 1)


class TestDeploy:
    def test_copies_all_four_files(self, tmp_path):
        written = deploy("joints", str(tmp_path / "out"))
        assert len(written) == 4
        assert all(p.is_file() for p in written)
        assert {p.name for p in written} == {f.name for f in context_files("joints")}

    def test_refuses_to_clobber_without_overwrite(self, tmp_path):
        out = str(tmp_path / "out")
        deploy("basic", out)
        with pytest.raises(FileExistsError):
            deploy("basic", out)

    def test_overwrite_allows_a_repeat(self, tmp_path):
        out = str(tmp_path / "out")
        deploy("basic", out)
        assert len(deploy("basic", out, overwrite=True)) == 4

    def test_unknown_context_rejected(self, tmp_path):
        with pytest.raises(RSIConfigError):
            deploy("nonesuch", str(tmp_path / "out"))
