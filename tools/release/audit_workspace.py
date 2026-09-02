#!/usr/bin/env python3
"""Run release-oriented checks that do not require third-party Python modules."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from xml.etree import ElementTree


CORE_PACKAGES = {
    "aurora_math",
    "aurora_map",
    "aurora_search",
    "aurora_trajectory",
    "aurora_planner_core",
    "aurora_prediction",
    "aurora_tracking",
    "aurora_risk",
}

REQUIRED_PACKAGES = CORE_PACKAGES | {
    "aurora_msgs",
    "aurora_ros",
    "aurora_bringup",
    "aurora_sim",
    "aurora_flight_adapter",
}

REQUIRED_DOCUMENTS = {
    "CHANGELOG.md",
    "LICENSE",
    "NOTICE",
    "README.md",
    "docs/api-compatibility.md",
    "docs/architecture.md",
    "docs/baseline.md",
    "docs/benchmarks.md",
    "docs/dependencies.md",
    "docs/parameters.md",
    "docs/simulation.md",
    "docs/external-backends.md",
    "docs/release-checklist.md",
    "docs/third-party-licenses.md",
    "docs/source-sbom.spdx.json",
}


def add_unique(items: list[str], value: str) -> None:
    if value not in items:
        items.append(value)


def audit_packages(root: Path, errors: list[str], warnings: list[str]) -> int:
    package_root = root / "src"
    manifests = sorted(package_root.glob("*/package.xml"))
    if not manifests:
        add_unique(errors, "no src/*/package.xml manifests found")
        return 0

    names: set[str] = set()
    versions: set[str] = set()
    for manifest in manifests:
        try:
            package = ElementTree.parse(manifest).getroot()
        except ElementTree.ParseError as error:
            add_unique(errors, f"{manifest}: invalid XML: {error}")
            continue

        name = package.findtext("name", default="").strip()
        if not name:
            add_unique(errors, f"{manifest}: missing package name")
            continue
        names.add(name)
        if name != manifest.parent.name:
            add_unique(errors, f"{manifest}: package name does not match directory")
        version = package.findtext("version", default="").strip()
        if version:
            versions.add(version)
        for field in ("version", "description", "maintainer"):
            if not package.findtext(field, default="").strip():
                add_unique(errors, f"{manifest}: missing {field}")

        license_name = package.findtext("license", default="").strip()
        if not license_name:
            add_unique(errors, f"{manifest}: missing license")
        elif license_name.upper() == "TODO":
            add_unique(warnings, f"{manifest}: license is still TODO")

    missing_required = sorted(REQUIRED_PACKAGES - names)
    for package in missing_required:
        add_unique(errors, f"missing required package manifest: {package}")
    if len(versions) > 1:
        add_unique(errors, f"AURORA package versions are inconsistent: {sorted(versions)}")
    return len(manifests)


def audit_documents(root: Path, errors: list[str]) -> None:
    for relative_path in sorted(REQUIRED_DOCUMENTS):
        path = root / relative_path
        if not path.is_file():
            add_unique(errors, f"missing required document: {relative_path}")


def audit_core_ros_independence(root: Path, errors: list[str]) -> None:
    include_pattern = re.compile(r"#\s*include\s*[<\"](?:rclcpp|sensor_msgs|tf2|tf2_ros)(?:/|[>\"])")
    for package in sorted(CORE_PACKAGES):
        source_root = root / "src" / package
        for source in sorted(source_root.rglob("*.cpp")) + sorted(source_root.rglob("*.hpp")):
            try:
                content = source.read_text(encoding="utf-8")
            except UnicodeDecodeError as error:
                add_unique(errors, f"{source}: cannot read as UTF-8: {error}")
                continue
            if include_pattern.search(content):
                add_unique(errors, f"{source}: core source includes a ROS adapter header")


def audit_ci(root: Path, errors: list[str]) -> None:
    workflow = root / ".github" / "workflows" / "ci.yml"
    if not workflow.is_file():
        add_unique(errors, "missing .github/workflows/ci.yml")
        return
    content = workflow.read_text(encoding="utf-8")
    required_fragments = {
        "ros_distro: humble": "Humble matrix entry",
        "ros_distro: jazzy": "Jazzy matrix entry",
        "DAURORA_BUILD_BENCHMARKS=ON": "benchmark build option",
        "--soft-risk": "soft-risk benchmark invocation",
        'risk_evaluations"] > 0': "soft-risk benchmark assertion",
        "src/aurora_flight_adapter/src": "flight-admission static-analysis source",
    }
    for fragment, description in required_fragments.items():
        if fragment not in content:
            add_unique(errors, f"CI missing {description}: {fragment}")


def audit_source_sbom(root: Path, errors: list[str]) -> None:
    path = root / "docs" / "source-sbom.spdx.json"
    if not path.is_file():
        return
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        add_unique(errors, f"{path}: invalid JSON: {error}")
        return
    if document.get("spdxVersion") != "SPDX-2.3":
        add_unique(errors, f"{path}: expected SPDX-2.3")
    package_names = {
        package.get("name")
        for package in document.get("packages", [])
        if isinstance(package, dict)
    }
    manifest_names = {
        manifest.parent.name
        for manifest in (root / "src").glob("*/package.xml")
    }
    if not manifest_names.issubset(package_names):
        add_unique(errors, f"{path}: missing source package entries")


def run_audit(root: Path, strict_licenses: bool) -> dict[str, object]:
    errors: list[str] = []
    warnings: list[str] = []
    package_count = audit_packages(root, errors, warnings)
    audit_documents(root, errors)
    audit_core_ros_independence(root, errors)
    audit_ci(root, errors)
    audit_source_sbom(root, errors)
    if strict_licenses:
        errors.extend(
            warning
            for warning in warnings
            if "license is still TODO" in warning
        )
    return {
        "status": "PASS" if not errors else "FAIL",
        "strict_licenses": strict_licenses,
        "package_count": package_count,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="AURORA-Planner repository root",
    )
    parser.add_argument(
        "--strict-licenses",
        action="store_true",
        help="Treat temporary TODO license fields as errors",
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    args = parser.parse_args()
    report = run_audit(args.root.resolve(), args.strict_licenses)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"workspace audit: {report['status']}")
        print(f"packages: {report['package_count']}")
        for warning in report["warnings"]:
            print(f"warning: {warning}")
        for error in report["errors"]:
            print(f"error: {error}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
