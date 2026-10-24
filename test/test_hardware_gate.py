from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
from pathlib import Path

import pytest


HARDWARE_ENVIRONMENT = (
    "WUJI_HARDWARE_TESTS",
    "WUJI_MUTATING_HARDWARE_TESTS",
    "WUJI_HARDWARE_SERIAL",
    "WUJI_HARDWARE_ENDPOINT",
    "WUJI_HARDWARE_SAFETY_ACK",
    "WUJI_HARDWARE_RUNNER",
    "WUJI_GLOVE_HARDWARE_TESTS",
    "WUJI_GLOVE_HARDWARE_SERIAL",
    "WUJI_GLOVE_HARDWARE_ENDPOINT",
    "WUJI_GLOVE_HARDWARE_HAND_SIDE",
    "WUJI_GLOVE_HARDWARE_RUNNER",
)
SAFETY_ACK = "fixture-unloaded-operator-state-not-restored"


def cpp_lexical_view(source: str) -> tuple[str, list[tuple[int, int]]]:
    output: list[str] = []
    raw_string_line_ranges: list[tuple[int, int]] = []
    index = 0
    state = "code"
    while index < len(source):
        current = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "code":
            raw_prefix = next(
                (
                    prefix
                    for prefix in ("u8R\"", "uR\"", "UR\"", "LR\"", "R\"")
                    if source.startswith(prefix, index)
                ),
                None,
            )
            if raw_prefix is not None:
                delimiter_start = index + len(raw_prefix)
                open_parenthesis = source.find(
                    "(", delimiter_start, delimiter_start + 17
                )
                delimiter = source[delimiter_start:open_parenthesis]
                if open_parenthesis >= 0 and re.fullmatch(
                    r"[^\s()\\]{0,16}", delimiter
                ):
                    marker = ")" + delimiter + '"'
                    literal_end = source.find(marker, open_parenthesis + 1)
                    if literal_end >= 0:
                        literal_end += len(marker)
                    else:
                        literal_end = len(source)
                    start_line = source.count("\n", 0, index)
                    end_line = source.count("\n", 0, literal_end)
                    raw_string_line_ranges.append((start_line, end_line))
                    output.extend(
                        "\n" if character == "\n" else " "
                        for character in source[index:literal_end]
                    )
                    index = literal_end
                    continue
            if current == "/" and following == "/":
                output.extend("  ")
                index += 2
                state = "line-comment"
                continue
            if current == "/" and following == "*":
                output.extend("  ")
                index += 2
                state = "block-comment"
                continue
            if current in ('"', "'"):
                output.append(" ")
                index += 1
                state = "string" if current == '"' else "character"
                continue
            output.append(current)
            index += 1
            continue
        if state == "line-comment":
            output.append("\n" if current == "\n" else " ")
            index += 1
            if current == "\n":
                state = "code"
            continue
        if state == "block-comment":
            if current == "*" and following == "/":
                output.extend("  ")
                index += 2
                state = "code"
            else:
                output.append("\n" if current == "\n" else " ")
                index += 1
            continue
        if current == "\\":
            output.append(" ")
            if following:
                output.append("\n" if following == "\n" else " ")
                index += 2
            else:
                index += 1
            continue
        output.append("\n" if current == "\n" else " ")
        index += 1
        if (state == "string" and current == '"') or (
            state == "character" and current == "'"
        ):
            state = "code"
    return "".join(output), raw_string_line_ranges


def known_preprocessor_condition(expression: str) -> bool | None:
    normalized = re.sub(r"[()\s]", "", expression)
    if re.fullmatch(r"0[uUlL]*", normalized):
        return False
    if re.fullmatch(r"1[uUlL]*", normalized):
        return True
    return None


def preprocessor_source_views(source: str) -> tuple[str, str]:
    stripped, raw_string_line_ranges = cpp_lexical_view(source)
    original_lines = source.splitlines(keepends=True)
    stripped_lines = stripped.splitlines(keepends=True)
    active_lines: list[str] = []
    forbidden_lines: list[str] = []
    conditionals: list[dict[str, bool | str]] = []

    def currently_active() -> bool:
        if not conditionals:
            return True
        return bool(conditionals[-1]["active"])

    index = 0
    while index < len(stripped_lines):
        line = stripped_lines[index]
        if not line.lstrip().startswith("#"):
            forbidden_lines.append(line)
            active_lines.append(line if currently_active() else "\n")
            index += 1
            continue

        last = index
        while last < len(original_lines) - 1:
            raw_end = max(
                (
                    end
                    for start, end in raw_string_line_ranges
                    if index <= start <= last and end > last
                ),
                default=last,
            )
            if raw_end > last:
                last = raw_end
                continue
            if original_lines[last].rstrip("\r\n").endswith("\\"):
                last += 1
                continue
            break
        directive_lines = stripped_lines[index : last + 1]
        logical = re.sub(r"\\\r?\n", " ", "".join(directive_lines))
        directive_match = re.match(
            r"\s*#\s*([A-Za-z_]\w*)\b(.*)", logical, re.DOTALL
        )
        keyword = directive_match.group(1) if directive_match else ""
        argument = directive_match.group(2).strip() if directive_match else ""
        active_lines.extend("\n" for _ in directive_lines)
        if keyword not in ("include", "include_next", "import"):
            forbidden_lines.append(logical)

        if keyword in ("if", "ifdef", "ifndef"):
            parent_active = currently_active()
            condition = (
                known_preprocessor_condition(argument)
                if keyword == "if"
                else None
            )
            branch = (
                "true" if condition is True else "false" if condition is False else "unknown"
            )
            conditionals.append(
                {
                    "parent_active": parent_active,
                    "branch": branch,
                    "active": parent_active and condition is True,
                }
            )
        elif keyword == "elif" and conditionals:
            frame = conditionals[-1]
            if frame["branch"] in ("true", "done"):
                frame["branch"] = "done"
                frame["active"] = False
            elif frame["branch"] == "unknown":
                frame["active"] = False
            else:
                condition = known_preprocessor_condition(argument)
                frame["branch"] = (
                    "true"
                    if condition is True
                    else "false"
                    if condition is False
                    else "unknown"
                )
                frame["active"] = bool(frame["parent_active"]) and condition is True
        elif keyword == "else" and conditionals:
            frame = conditionals[-1]
            if frame["branch"] == "false":
                frame["active"] = bool(frame["parent_active"])
            else:
                frame["active"] = False
            frame["branch"] = "done"
        elif keyword == "endif" and conditionals:
            conditionals.pop()
        index = last + 1

    return "".join(active_lines), "".join(forbidden_lines)


def cpp_tokens(source: str) -> list[str]:
    return re.findall(r"[A-Za-z_]\w*|::|[=().,;]", source)


def direct_raw_sdk_references(source: str) -> set[str]:
    _, forbidden_code = preprocessor_source_views(source)
    tokens = cpp_tokens(forbidden_code)
    return {
        token
        for index, token in enumerate(tokens)
        if token.startswith("wuji_")
        and (index + 1 == len(tokens) or tokens[index + 1] != "::")
    }


def production_glove_session_usage(source: str) -> bool:
    active_code, _ = preprocessor_source_views(source)
    tokens = cpp_tokens(active_code)
    production_prefix = ["auto"]
    production_suffix = ["=", "sdk", "::", "GloveSdkApi", "::", "production", "(", ")"]
    api_names: set[str] = set()
    for index in range(len(tokens) - len(production_suffix) - 1):
        if (
            tokens[index : index + len(production_prefix)] == production_prefix
            and tokens[index + 2 : index + 2 + len(production_suffix)] == production_suffix
        ):
            api_names.add(tokens[index + 1])

    for api_name in api_names:
        session_prefix = ["sdk", "::", "GloveSession"]
        for index in range(len(tokens) - 6):
            if tokens[index : index + 3] != session_prefix:
                continue
            session_name = tokens[index + 3]
            if tokens[index + 4 : index + 7] != ["(", api_name, ","]:
                continue
            connect = [session_name, ".", "connect", "("]
            device = [session_name, ".", "device", "("]
            has_connect = any(
                tokens[position : position + 4] == connect
                for position in range(len(tokens) - 3)
            )
            has_device = any(
                tokens[position : position + 4] == device
                for position in range(len(tokens) - 3)
            )
            if has_connect and has_device:
                return True
    return False


def clean_environment(**updates: str) -> dict[str, str]:
    environment = os.environ.copy()
    for name in HARDWARE_ENVIRONMENT:
        environment.pop(name, None)
    environment.update(updates)
    return environment


def run_gate(mode: str, **updates: str) -> subprocess.CompletedProcess[str]:
    root = Path(__file__).resolve().parents[1]
    return subprocess.run(
        [root / "scripts/run_hardware_tests.sh", mode],
        check=False,
        capture_output=True,
        env=clean_environment(**updates),
        text=True,
    )


def runner_for(marker: Path, exit_code: int = 0) -> str:
    return (
        f"touch -- {shlex.quote(str(marker))}"
        + (f"; exit {exit_code}" if exit_code else "")
    )


def test_hardware_gate_is_opt_in(tmp_path: Path) -> None:
    marker = tmp_path / "called"
    result = run_gate("--read-only", WUJI_HARDWARE_RUNNER=runner_for(marker))
    assert result.returncode == 77
    assert not marker.exists()


def test_enabled_gate_requires_an_explicit_selector(tmp_path: Path) -> None:
    marker = tmp_path / "called"
    result = run_gate(
        "--read-only",
        WUJI_HARDWARE_TESTS="1",
        WUJI_HARDWARE_RUNNER=runner_for(marker),
    )
    assert result.returncode == 2
    assert not marker.exists()


@pytest.mark.parametrize(
    ("selector", "value"),
    [("WUJI_HARDWARE_SERIAL", "SN123"), ("WUJI_HARDWARE_ENDPOINT", "192.0.2.10")],
)
def test_read_only_gate_accepts_one_explicit_selector(
    tmp_path: Path, selector: str, value: str
) -> None:
    marker = tmp_path / "called"
    result = run_gate(
        "--read-only",
        WUJI_HARDWARE_TESTS="1",
        WUJI_HARDWARE_RUNNER=runner_for(marker),
        **{selector: value},
    )
    assert result.returncode == 0, result.stderr
    assert marker.exists()


def test_mutating_gate_requires_both_opt_ins_and_exact_ack(tmp_path: Path) -> None:
    marker = tmp_path / "called"
    base = {
        "WUJI_HARDWARE_TESTS": "1",
        "WUJI_HARDWARE_SERIAL": "SN123",
        "WUJI_HARDWARE_RUNNER": runner_for(marker),
    }
    assert run_gate("--mutating", **base).returncode == 3
    base["WUJI_MUTATING_HARDWARE_TESTS"] = "1"
    assert run_gate("--mutating", **base).returncode == 3
    base["WUJI_HARDWARE_SAFETY_ACK"] = SAFETY_ACK
    assert run_gate("--mutating", **base).returncode == 0
    assert marker.exists()


@pytest.mark.parametrize(
    "selectors",
    [
        {"WUJI_HARDWARE_ENDPOINT": "192.0.2.10"},
        {
            "WUJI_HARDWARE_SERIAL": "SN123",
            "WUJI_HARDWARE_ENDPOINT": "192.0.2.10",
        },
    ],
)
def test_mutating_gate_rejects_nonserial_or_ambiguous_selection(
    tmp_path: Path, selectors: dict[str, str]
) -> None:
    marker = tmp_path / "called"
    result = run_gate(
        "--mutating",
        WUJI_HARDWARE_TESTS="1",
        WUJI_MUTATING_HARDWARE_TESTS="1",
        WUJI_HARDWARE_SAFETY_ACK=SAFETY_ACK,
        WUJI_HARDWARE_RUNNER=runner_for(marker),
        **selectors,
    )
    assert result.returncode == 2
    assert not marker.exists()


def test_gate_rejects_malformed_selector_before_runner(tmp_path: Path) -> None:
    marker = tmp_path / "called"
    result = run_gate(
        "--read-only",
        WUJI_HARDWARE_TESTS="1",
        WUJI_HARDWARE_SERIAL="SN 123",
        WUJI_HARDWARE_RUNNER=runner_for(marker),
    )
    assert result.returncode == 2
    assert not marker.exists()


def test_runner_failure_status_is_preserved(tmp_path: Path) -> None:
    marker = tmp_path / "called"
    result = run_gate(
        "--read-only",
        WUJI_HARDWARE_TESTS="1",
        WUJI_HARDWARE_SERIAL="SN123",
        WUJI_HARDWARE_RUNNER=runner_for(marker, 41),
    )
    assert result.returncode == 41
    assert marker.exists()


def test_gate_does_not_search_outside_repository_root(tmp_path: Path) -> None:
    outer_root = tmp_path / "outer"
    script_directory = outer_root / "repository/scripts"
    script_directory.mkdir(parents=True)
    script = script_directory / "run_hardware_tests.sh"
    source = Path(__file__).resolve().parents[1] / "scripts/run_hardware_tests.sh"
    shutil.copy2(source, script)

    marker = tmp_path / "outside-repository-called"
    executable = (
        outer_root
        / "build/wuji_hand2_driver/wuji_hand2_hardware_read_only"
    )
    executable.parent.mkdir(parents=True)
    executable.write_text(
        "#!/usr/bin/env bash\n" f"touch -- {shlex.quote(str(marker))}\n"
    )
    executable.chmod(0o755)

    result = subprocess.run(
        [script, "--read-only"],
        check=False,
        capture_output=True,
        cwd=tmp_path,
        env=clean_environment(
            WUJI_HARDWARE_TESTS="1", WUJI_HARDWARE_SERIAL="SN123"
        ),
        text=True,
    )

    assert result.returncode == 2
    assert not marker.exists()


def test_read_only_source_uses_only_nonmutating_hand2_api_calls() -> None:
    root = Path(__file__).resolve().parents[1]
    source = (
        root / "wuji_hand2_driver/test/hardware/test_read_only_smoke.cpp"
    ).read_text()
    assert "api.subscribe_fingertip[finger]" in source
    for call in (
        "api.enable(",
        "api.disable(",
        "api.clear_fault(",
        "api.emergency_stop(",
        "api.set_effort_limit(",
        "api.set_effort_limits(",
        "api.set_mit_params(",
        "api.tactile_calibrate(",
        "api.tactile_status(",
        "api.open_joint_command_publisher(",
        "api.send_joint_commands(",
    ):
        assert call not in source


def test_glove_hardware_gate_is_independently_opt_in(tmp_path: Path) -> None:
    marker = tmp_path / "glove-called"
    result = run_gate(
        "--glove-read-only", WUJI_GLOVE_HARDWARE_RUNNER=runner_for(marker)
    )
    assert result.returncode == 77
    assert not marker.exists()


@pytest.mark.parametrize(
    "selection",
    [
        {},
        {"WUJI_GLOVE_HARDWARE_SERIAL": "WUJKSN123"},
        {"WUJI_GLOVE_HARDWARE_ENDPOINT": "192.0.2.11"},
        {"WUJI_GLOVE_HARDWARE_HAND_SIDE": "left"},
    ],
)
def test_glove_gate_accepts_unique_discovery_and_each_selector(
    tmp_path: Path, selection: dict[str, str]
) -> None:
    marker = tmp_path / "glove-called"
    result = run_gate(
        "--glove-read-only",
        WUJI_GLOVE_HARDWARE_TESTS="1",
        WUJI_GLOVE_HARDWARE_RUNNER=runner_for(marker),
        **selection,
    )
    assert result.returncode == 0, result.stderr
    assert marker.exists()


def test_glove_gate_does_not_enable_hand2_runner(tmp_path: Path) -> None:
    marker = tmp_path / "called"
    result = run_gate(
        "--glove-read-only",
        WUJI_HARDWARE_TESTS="1",
        WUJI_GLOVE_HARDWARE_RUNNER=runner_for(marker),
    )
    assert result.returncode == 77
    assert not marker.exists()


def test_glove_gate_rejects_malformed_selector_before_runner(tmp_path: Path) -> None:
    marker = tmp_path / "called"
    result = run_gate(
        "--glove-read-only",
        WUJI_GLOVE_HARDWARE_TESTS="1",
        WUJI_GLOVE_HARDWARE_SERIAL="SN 123",
        WUJI_GLOVE_HARDWARE_RUNNER=runner_for(marker),
    )
    assert result.returncode == 2
    assert not marker.exists()


def test_glove_read_only_source_uses_only_production_api_and_session() -> None:
    root = Path(__file__).resolve().parents[1]
    hardware_directory = root / "wuji_glove_driver/test/hardware"
    source = "\n".join(
        (hardware_directory / name).read_text()
        for name in (
            "test_read_only_smoke.cpp",
            "read_only_stream_probe.hpp",
            "hardware_test_config.hpp",
            "hardware_test_validation.hpp",
        )
    )
    assert production_glove_session_usage(source)
    assert direct_raw_sdk_references(source) == set()


def test_glove_source_analysis_ignores_comments_and_literals() -> None:
    spoofed = r'''
      // auto api = sdk::GloveSdkApi::production();
      // sdk::GloveSession session(api, hooks);
      // session.connect(config); session.device();
      const char* text = "wuji_sub_set_push_rate(subscription, 10)";
      const char* raw = R"fixture(
        auto api = sdk::GloveSdkApi::production();
        sdk::GloveSession session(api, hooks);
        session.connect(config); session.device();
        wuji_sub_set_push_rate(subscription, 10);
      )fixture";
    '''
    assert not production_glove_session_usage(spoofed)
    assert direct_raw_sdk_references(spoofed) == set()


def test_glove_source_analysis_rejects_direct_unlisted_sdk_reference() -> None:
    unsafe = "wuji_sub_set_push_rate(subscription, 10);"
    assert direct_raw_sdk_references(unsafe) == {"wuji_sub_set_push_rate"}


def test_glove_source_analysis_rejects_positive_spoof_inside_if_zero() -> None:
    inactive = """
      #if 0
      auto api = sdk::GloveSdkApi::production();
      sdk::GloveSession session(api, hooks);
      session.connect(config); session.device();
      #endif
    """
    assert not production_glove_session_usage(inactive)


def test_glove_source_analysis_rejects_positive_spoof_in_continued_macro() -> None:
    macro = """
      #define PRETEND_SESSION() \\
        auto api = sdk::GloveSdkApi::production(); \\
        sdk::GloveSession session(api, hooks); \\
        session.connect(config); session.device()
      PRETEND_SESSION();
    """
    assert not production_glove_session_usage(macro)


def test_glove_source_analysis_rejects_unknown_conditional_positives() -> None:
    unknown = """
      #ifdef BUILD_FAKE_SESSION
      auto api = sdk::GloveSdkApi::production();
      sdk::GloveSession session(api, hooks);
      session.connect(config); session.device();
      #else
      auto api = sdk::GloveSdkApi::production();
      sdk::GloveSession session(api, hooks);
      session.connect(config); session.device();
      #endif
    """
    assert not production_glove_session_usage(unknown)


def test_glove_source_analysis_tracks_nested_known_conditionals() -> None:
    nested = """
      #if 0
        #if 1
        auto fake_api = sdk::GloveSdkApi::production();
        sdk::GloveSession fake_session(fake_api, hooks);
        fake_session.connect(config); fake_session.device();
        #endif
      #else
        #if 1
        auto api = sdk::GloveSdkApi::production();
        sdk::GloveSession session(api, hooks);
        session.connect(config); session.device();
        #endif
      #endif
    """
    assert production_glove_session_usage(nested)


def test_glove_source_analysis_scans_inactive_and_continued_macros_for_raw_sdk() -> None:
    unsafe = """
      #if 0
      wuji_glove_reboot(device);
      #endif
      #define SET_RATE(subscription) \\
        wuji_sub_set_push_rate( \\
          subscription, 10)
    """
    assert direct_raw_sdk_references(unsafe) == {
        "wuji_glove_reboot",
        "wuji_sub_set_push_rate",
    }


def test_glove_source_analysis_rejects_positive_spoof_after_multiline_raw_macro() -> None:
    positive = (
        "auto api = sdk::GloveSdkApi::production(); "
        "sdk::GloveSession session(api, hooks); "
        "session.connect(config); session.device()"
    )
    macro = (
        '      #define SPOOF R"tag(\n'
        "        ignored raw contents\n"
        f'      )tag" {positive}\n'
        "      SPOOF;\n"
    )
    assert not production_glove_session_usage(macro)


def test_glove_source_analysis_scans_multiline_raw_macro_for_raw_sdk() -> None:
    unsafe = r'''
      #define RAW_COMMAND R"d!m(
        ignored raw contents
      )d!m" wuji_sub_set_push_rate(subscription, 10)
    '''
    assert direct_raw_sdk_references(unsafe) == {"wuji_sub_set_push_rate"}
