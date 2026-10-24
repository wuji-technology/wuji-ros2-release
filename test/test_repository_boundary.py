from __future__ import annotations

import re
from pathlib import Path

import pytest


PUBLIC_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_NAMES = (
    "wuji_bringup",
    "wuji_glove_driver",
    "wuji_hand2_driver",
    "wuji_msgs",
    "wuji_sdk_vendor",
    "wuji_tools",
)
GLOVE_PARAMETER_ROWS = (
    ("endpoint", "string", '""'),
    ("serial_number", "string", '""'),
    ("hand_side", "string", '""'),
    ("connect_timeout", "double", "10.0"),
    ("publish_emf_poses", "bool", "true"),
    ("publish_tip_poses", "bool", "true"),
    ("publish_skeleton", "bool", "true"),
    ("publish_imu", "bool", "true"),
    ("publish_tactile", "bool", "true"),
    ("publish_tactile_residual", "bool", "true"),
    ("publish_tactile_binary", "bool", "true"),
    ("publish_tactile_zones", "bool", "true"),
    ("publish_tactile_points", "bool", "true"),
)
GLOVE_TOPIC_ROWS = (
    (
        "joint_angles",
        "wuji_msgs/msg/HandJointAngles",
        "Required",
        "Always enabled",
    ),
    (
        "emf_poses",
        "wuji_msgs/msg/TrackedPoseArray",
        "Optional",
        "publish_emf_poses",
    ),
    (
        "tip_poses",
        "wuji_msgs/msg/TrackedPoseArray",
        "Optional",
        "publish_tip_poses",
    ),
    (
        "hand_skeleton",
        "wuji_msgs/msg/TrackedPoseArray",
        "Optional",
        "publish_skeleton",
    ),
    ("imu/palm", "sensor_msgs/msg/Imu", "Optional", "publish_imu"),
    (
        "tactile",
        "wuji_msgs/msg/TactileGrid",
        "Optional",
        "publish_tactile",
    ),
    (
        "tactile/residual",
        "wuji_msgs/msg/TactileGrid",
        "Optional",
        "publish_tactile_residual",
    ),
    (
        "tactile/binary",
        "wuji_msgs/msg/TactileGrid",
        "Optional",
        "publish_tactile_binary",
    ),
    (
        "tactile/zones",
        "wuji_msgs/msg/TactileZoneArray",
        "Optional",
        "publish_tactile_zones",
    ),
    (
        "tactile/points",
        "sensor_msgs/msg/PointCloud2",
        "Optional",
        "publish_tactile_points",
    ),
)


def visible_markdown_lines(document: str) -> tuple[str | None, ...]:
    visible: list[str | None] = []
    fence_character = ""
    fence_length = 0
    opening_fence = re.compile(r"^ {0,3}(`{3,}|~{3,})(.*)$")
    closing_fence = re.compile(r"^ {0,3}([`~]+)[ \t]*$")

    for line in document.splitlines():
        if fence_character:
            closing = closing_fence.fullmatch(line)
            if closing:
                marker = closing.group(1)
                if (
                    len(marker) >= fence_length
                    and all(character == fence_character for character in marker)
                ):
                    fence_character = ""
                    fence_length = 0
            visible.append(None)
            continue

        opening = opening_fence.fullmatch(line)
        if opening:
            marker = opening.group(1)
            info_string = opening.group(2)
            if marker[0] == "~" or "`" not in info_string:
                fence_character = marker[0]
                fence_length = len(marker)
                visible.append(None)
                continue
        visible.append(line)
    return tuple(visible)


def parse_markdown_table(
    document: str,
    heading: str,
    next_heading: str,
    header: tuple[str, ...],
) -> tuple[tuple[str, ...], ...]:
    lines = visible_markdown_lines(document)
    heading_indices = [index for index, line in enumerate(lines) if line == heading]
    next_heading_indices = [
        index for index, line in enumerate(lines) if line == next_heading
    ]
    assert len(heading_indices) == 1
    assert len(next_heading_indices) == 1
    section_start = heading_indices[0] + 1
    section_end = next_heading_indices[0]
    assert section_start <= section_end

    table_headers = []
    header_line = "| " + " | ".join(header) + " |"
    for index in range(section_start, section_end):
        line = lines[index]
        if line == header_line:
            table_headers.append(index)
    assert len(table_headers) == 1

    header_index = table_headers[0]
    separator = lines[header_index + 1]
    assert separator is not None
    separator_cells = [cell.strip() for cell in separator.strip("|").split("|")]
    assert len(separator_cells) == len(header)
    assert all(re.fullmatch(r":?-{3,}:?", cell) for cell in separator_cells)

    rows = []
    for line in lines[header_index + 2 : section_end]:
        if line is None or not line.startswith("|"):
            break
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        assert len(cells) == len(header)
        rows.append(
            tuple(
                cell[1:-1]
                if len(cell) >= 2 and cell.startswith("`") and cell.endswith("`")
                else cell
                for cell in cells
            )
        )
    assert rows
    return tuple(rows)


def assert_glove_readme_contract(readme: str) -> None:
    parameter_rows = parse_markdown_table(
        readme,
        "## Parameters",
        "## Topics",
        ("Parameter", "Type", "Default", "Description"),
    )
    assert tuple(row[:3] for row in parameter_rows) == GLOVE_PARAMETER_ROWS

    topic_rows = parse_markdown_table(
        readme,
        "## Topics",
        "## Fault and Shutdown Behavior",
        ("Topic", "Type", "Policy", "Enable Parameter"),
    )
    assert topic_rows == GLOVE_TOPIC_ROWS



def test_public_packages_have_readmes() -> None:
    for package_name in PACKAGE_NAMES:
        readme = PUBLIC_ROOT / package_name / "README.md"
        assert readme.is_file()
        assert readme.read_text().startswith(f"# {package_name}\n")


def test_public_package_set_is_exact() -> None:
    package_names = {
        path.parent.name for path in PUBLIC_ROOT.glob("*/package.xml")
    }
    assert package_names == set(PACKAGE_NAMES)


def test_glove_readme_documents_exact_parameters_and_topics() -> None:
    readme = (PUBLIC_ROOT / "wuji_glove_driver/README.md").read_text()
    assert_glove_readme_contract(readme)


@pytest.mark.parametrize(
    ("original", "replacement"),
    [
        (
            "| `connect_timeout` | double | `10.0` |",
            "| `connect_timeout` | string | `10.0` |",
        ),
        (
            "| `connect_timeout` | double | `10.0` |",
            "| `connect_timeout` | double | `5.0` |",
        ),
        (
            "| `joint_angles` | `wuji_msgs/msg/HandJointAngles` | Required | Always enabled |",
            "| `joint_angles` | `sensor_msgs/msg/JointState` | Required | Always enabled |",
        ),
        (
            "| `joint_angles` | `wuji_msgs/msg/HandJointAngles` | Required | Always enabled |",
            "| `joint_angles` | `wuji_msgs/msg/HandJointAngles` | Optional | Always enabled |",
        ),
        (
            "| `joint_angles` | `wuji_msgs/msg/HandJointAngles` | Required | Always enabled |",
            "| `joint_angles` | `wuji_msgs/msg/HandJointAngles` | Required | "
            "`publish_joint_angles` |",
        ),
    ],
)
def test_glove_readme_contract_rejects_wrong_cell_values(
    original: str, replacement: str
) -> None:
    readme = (PUBLIC_ROOT / "wuji_glove_driver/README.md").read_text()
    assert readme.count(original) == 1
    with pytest.raises(AssertionError):
        assert_glove_readme_contract(readme.replace(original, replacement))


@pytest.mark.parametrize("mutation", ["missing", "extra", "duplicate"])
def test_glove_readme_contract_rejects_wrong_row_inventory(mutation: str) -> None:
    readme = (PUBLIC_ROOT / "wuji_glove_driver/README.md").read_text()
    row = "| `publish_tactile_points` | bool | `true` | Starts the tactile point-cloud stream |"
    assert readme.count(row) == 1
    if mutation == "missing":
        changed = readme.replace(f"{row}\n", "")
    elif mutation == "extra":
        changed = readme.replace(
            f"{row}\n",
            f"{row}\n| `publish_unknown` | bool | `true` | Unknown stream |\n",
        )
    else:
        changed = readme.replace(f"{row}\n", f"{row}\n{row}\n")
    with pytest.raises(AssertionError):
        assert_glove_readme_contract(changed)


def test_glove_readme_parser_ignores_prose_and_code_blocks() -> None:
    readme = (PUBLIC_ROOT / "wuji_glove_driver/README.md").read_text()
    decoy = """\
The `connect_timeout` value in this prose is intentionally irrelevant.

```text
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `connect_timeout` | string | `5.0` | Decoy |
```

"""
    assert_glove_readme_contract(
        readme.replace("## Parameters\n\n", f"## Parameters\n\n{decoy}", 1)
    )


@pytest.mark.parametrize(
    "decoy",
    [
        "```text\n## Parameters\n```\n\n",
        "   ```text\n"
        "   | Parameter | Type | Default | Description |\n"
        "   |-----------|------|---------|-------------|\n"
        "   | `connect_timeout` | string | `5.0` | Decoy |\n"
        "   ```\n\n",
        "~~~text\n"
        "| Parameter | Type | Default | Description |\n"
        "|-----------|------|---------|-------------|\n"
        "| `connect_timeout` | string | `5.0` | Decoy |\n"
        "~~~~\n\n",
    ],
)
def test_glove_readme_parser_ignores_commonmark_fenced_decoys(
    decoy: str,
) -> None:
    readme = (PUBLIC_ROOT / "wuji_glove_driver/README.md").read_text()
    assert_glove_readme_contract(
        readme.replace("## Parameters\n\n", f"## Parameters\n\n{decoy}", 1)
    )


@pytest.mark.parametrize("mutation", ["duplicate-heading", "duplicate-table"])
def test_glove_readme_parser_rejects_visible_duplicate_contracts(
    mutation: str,
) -> None:
    readme = (PUBLIC_ROOT / "wuji_glove_driver/README.md").read_text()
    if mutation == "duplicate-heading":
        changed = readme.replace(
            "## Parameters\n\n",
            "## Parameters\n\nDuplicate section.\n\n## Parameters\n\n",
            1,
        )
    else:
        decoy = (
            "| Parameter | Type | Default | Description |\n"
            "|-----------|------|---------|-------------|\n"
            "| `connect_timeout` | string | `5.0` | Visible duplicate |\n\n"
        )
        changed = readme.replace(
            "## Parameters\n\n", f"## Parameters\n\n{decoy}", 1
        )
    with pytest.raises(AssertionError):
        assert_glove_readme_contract(changed)


def test_glove_readme_parser_hides_unclosed_fence_to_end_of_file() -> None:
    readme = (PUBLIC_ROOT / "wuji_glove_driver/README.md").read_text()
    changed = readme.replace("## Parameters\n", "```text\n## Parameters\n", 1)
    with pytest.raises(AssertionError):
        assert_glove_readme_contract(changed)


def test_public_readme_keeps_the_user_facing_structure() -> None:
    readme = (PUBLIC_ROOT / "README.md").read_text()
    headings = [
        line
        for line in readme.splitlines()
        if line.startswith("## ") and not line.startswith("### ")
    ]
    assert headings == [
        "## Supported Environment",
        "## Repository Structure",
        "## Quick Start",
        "## Packages",
        "## Troubleshooting",
        "## Contributing",
        "## Contact",
    ]
    assert "-DBUILD_TESTING=OFF" in readme
    assert "## CI Status" not in readme
    assert "## Citation" not in readme
    assert "## Appendix" not in readme


def find_forbidden_wuji_ros_apt_packages(paths: list[Path]) -> list[Path]:
    command = re.compile(
        r"\b(?:sudo\s+)?apt(?:-get)?\s+install\b(?P<arguments>[^\n]*)",
        flags=re.IGNORECASE,
    )
    package = re.compile(
        r"(?:ros-[a-z0-9_$\{\}-]+-wuji(?:[-_][a-z0-9_$\{\}-]+)*"
        r"|wuji[-_]ros2(?:[-_][a-z0-9_$\{\}-]+)*)",
        flags=re.IGNORECASE,
    )
    violations = []
    for path in paths:
        joined = re.sub(r"\\[ \t]*\n[ \t]*", " ", path.read_text())
        for match in command.finditer(joined):
            arguments = re.split(
                r"&&|\|\||[;|#]", match.group("arguments"), maxsplit=1
            )[0]
            if any(
                package.fullmatch(item.strip("`'\"(),."))
                for item in arguments.split()
            ):
                violations.append(path)
    return violations


def test_public_documentation_does_not_advertise_a_wuji_ros_apt_package() -> None:
    assert find_forbidden_wuji_ros_apt_packages(list(PUBLIC_ROOT.rglob("*.md"))) == []


def test_apt_package_check_detects_shell_line_continuations(tmp_path: Path) -> None:
    document = tmp_path / "README.md"
    document.write_text(
        "sudo apt-get install " + chr(92) + "\n  ros-lyrical-wuji-tools\n"
    )

    assert find_forbidden_wuji_ros_apt_packages([document]) == [document]
