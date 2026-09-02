#!/usr/bin/env python3
"""Generate or validate a source-package SPDX 2.3 inventory.

This inventory is intentionally limited to package.xml metadata. It does not
claim installed OS package versions, optional backend versions, or final
license conclusions.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree


def package_id(name: str) -> str:
    return "SPDXRef-Package-" + re.sub(r"[^A-Za-z0-9.-]", "-", name)


def read_manifests(root: Path) -> dict[str, ElementTree.Element]:
    manifests: dict[str, ElementTree.Element] = {}
    for path in sorted((root / "src").glob("*/package.xml")):
        package = ElementTree.parse(path).getroot()
        name = package.findtext("name", default="").strip()
        if not name:
            raise ValueError(f"{path}: missing package name")
        manifests[name] = package
    if not manifests:
        raise ValueError("no src/*/package.xml manifests found")
    return manifests


def direct_dependencies(package: ElementTree.Element) -> list[str]:
    names: set[str] = set()
    for tag in ("buildtool_depend", "build_depend", "exec_depend", "depend", "test_depend"):
        names.update(value.text.strip() for value in package.findall(tag) if value.text and value.text.strip())
    return sorted(names)


def creation_time(explicit: str | None) -> str:
    if explicit:
        return explicit
    epoch = os.environ.get("SOURCE_DATE_EPOCH")
    if epoch:
        return datetime.fromtimestamp(int(epoch), tz=timezone.utc).isoformat().replace("+00:00", "Z")
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def build_document(root: Path, created: str | None) -> dict[str, object]:
    manifests = read_manifests(root)
    names = set(manifests)
    dependency_names = {
        dependency
        for package in manifests.values()
        for dependency in direct_dependencies(package)
    }
    all_names = sorted(names | dependency_names)

    packages: list[dict[str, object]] = []
    for name in all_names:
        manifest = manifests.get(name)
        if manifest is None:
            packages.append(
                {
                    "SPDXID": package_id(name),
                    "name": name,
                    "versionInfo": "UNRESOLVED",
                    "downloadLocation": "NOASSERTION",
                    "filesAnalyzed": False,
                    "licenseConcluded": "NOASSERTION",
                    "licenseDeclared": "NOASSERTION",
                    "copyrightText": "NOASSERTION",
                }
            )
            continue

        version = manifest.findtext("version", default="UNRESOLVED").strip() or "UNRESOLVED"
        declared_license = manifest.findtext("license", default="").strip()
        declared_license = declared_license if declared_license and declared_license.upper() != "TODO" else "NOASSERTION"
        packages.append(
            {
                "SPDXID": package_id(name),
                "name": name,
                "versionInfo": version,
                "downloadLocation": "NOASSERTION",
                "filesAnalyzed": False,
                "licenseConcluded": "NOASSERTION",
                "licenseDeclared": declared_license,
                "copyrightText": "NOASSERTION",
            }
        )

    relationships: list[dict[str, str]] = []
    for name in sorted(names):
        for dependency in direct_dependencies(manifests[name]):
            relationships.append(
                {
                    "spdxElementId": package_id(name),
                    "relationshipType": "DEPENDS_ON",
                    "relatedSpdxElement": package_id(dependency),
                }
            )

    own_ids = [package_id(name) for name in sorted(names)]
    return {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": "AURORA-Planner source package dependency inventory",
        "documentNamespace": "https://spdx.org/spdxdocs/aurora-planner-source-0.1.0",
        "creationInfo": {
            "created": creation_time(created),
            "creators": ["Tool: AURORA-Planner source SBOM generator"],
        },
        "documentDescribes": own_ids,
        "packages": packages,
        "relationships": relationships,
    }


def validate_document(root: Path, document: dict[str, object]) -> None:
    expected = build_document(root, created="2000-01-01T00:00:00Z")
    if document.get("spdxVersion") != "SPDX-2.3":
        raise ValueError("SBOM is not SPDX-2.3")
    actual_packages = {item.get("name") for item in document.get("packages", []) if isinstance(item, dict)}
    expected_packages = {item["name"] for item in expected["packages"]}
    if actual_packages != expected_packages:
        raise ValueError("SBOM package set does not match current package.xml dependencies")
    actual_relationships = {
        (
            item.get("spdxElementId"),
            item.get("relationshipType"),
            item.get("relatedSpdxElement"),
        )
        for item in document.get("relationships", [])
        if isinstance(item, dict)
    }
    expected_relationships = {
        (
            item["spdxElementId"],
            item["relationshipType"],
            item["relatedSpdxElement"],
        )
        for item in expected["relationships"]
    }
    if actual_relationships != expected_relationships:
        raise ValueError("SBOM dependency relationships do not match current package.xml files")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="SPDX JSON output path",
    )
    parser.add_argument(
        "check_path",
        nargs="?",
        type=Path,
        help="Existing SPDX JSON path when used with --check",
    )
    parser.add_argument("--created", help="UTC SPDX creation timestamp")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate an existing SPDX file against current package.xml files",
    )
    args = parser.parse_args()
    root = args.root.resolve()
    output_argument = args.output or args.check_path or Path("docs/source-sbom.spdx.json")
    output = output_argument if output_argument.is_absolute() else root / output_argument
    try:
        if args.check:
            validate_document(root, json.loads(output.read_text(encoding="utf-8")))
            print(f"source SBOM: PASS ({output})")
        else:
            document = build_document(root, args.created)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(document, indent=2, sort_keys=False) + "\n", encoding="utf-8")
            print(f"source SBOM: wrote {output}")
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"source SBOM: FAIL: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
