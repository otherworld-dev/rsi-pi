"""The shipped RSI contexts must be resolvable, complete and parseable.

These guard the packaging: the context files are data, so a build that
forgets to include them produces a library that imports cleanly and then
cannot run at all.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from RSIPI import (
    context,
    context_files,
    available_contexts,
    describe_contexts,
    DEFAULT_CONTEXT,
)
from RSIPI.config_parser import ConfigParser
from RSIPI.exceptions import RSIConfigError


class TestContextResolution:
    def test_default_is_joints(self):
        # joints is the most capable context that works on ANY 6-axis robot;
        # full is deliberately not the default because its external-axis
        # objects cannot bind without external axes.
        assert DEFAULT_CONTEXT == "joints"

    def test_context_with_no_argument_returns_the_default(self):
        assert context() == context(DEFAULT_CONTEXT)

    @pytest.mark.parametrize("name", available_contexts())
    def test_every_context_resolves_to_an_existing_config(self, name):
        assert Path(context(name)).is_file()

    @pytest.mark.parametrize("name", available_contexts())
    def test_every_context_ships_all_four_files(self, name):
        files = context_files(name)
        assert len(files) == 4
        assert all(f.is_file() for f in files)

    @pytest.mark.parametrize("name", available_contexts())
    def test_every_context_file_is_wellformed_xml(self, name):
        for f in context_files(name):
            ET.parse(f)

    @pytest.mark.parametrize("name", available_contexts())
    def test_every_config_parses_into_variables(self, name):
        parser = ConfigParser(context(name))
        assert parser.get_network_settings()["port"]
        assert parser.send_variables and parser.receive_variables

    def test_name_is_case_and_space_insensitive(self):
        assert context("  JOINTS ") == context("joints")

    def test_unknown_name_raises_and_lists_the_options(self):
        with pytest.raises(RSIConfigError) as excinfo:
            context("nonesuch")
        message = str(excinfo.value)
        for name in available_contexts():
            assert name in message

    def test_context_files_rejects_unknown_names_too(self):
        with pytest.raises(RSIConfigError):
            context_files("nonesuch")


class TestContextContents:
    """Spot-check the properties each context is chosen for."""

    def test_joints_declares_joint_corrections(self):
        assert "AKorr" in ConfigParser(context("joints")).receive_variables

    def test_basic_has_no_joint_corrections(self):
        assert "AKorr" not in ConfigParser(context("basic")).receive_variables

    def test_only_onlysend_sets_the_onlysend_flag(self):
        for name in available_contexts():
            settings = ConfigParser(context(name)).get_network_settings()
            assert settings["onlysend"] is (name == "onlysend")

    def test_only_full_declares_external_axes(self):
        for name in available_contexts():
            declared = ConfigParser(context(name)).receive_variables
            assert ("EKorr" in declared) is (name == "full")

    def test_only_stop_declares_movestop(self):
        for name in available_contexts():
            declared = ConfigParser(context(name)).receive_variables
            assert ("MoveStop" in declared) is (name == "stop")

    def test_describe_lists_every_context(self):
        text = describe_contexts()
        for name in available_contexts():
            assert name in text
