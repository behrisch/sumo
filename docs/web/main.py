#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2010-2026 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    main.py
# @author  Angelo Banse
# @date    2026-02-13

"""
This script is used by the mkdocs-macros-plugin to list all markdown files inside a directory (even with subfolders)

Usage (only when called from a page inside the folder whose contents we want to list):
  {{ list_pages("Netedit") }}
  {{ list_pages("Contributed", recursive=true) }}
"""

from pathlib import Path
import re


def define_env(env):
    docs_dir = Path(env.conf["docs_dir"]).resolve()
    use_dir_urls = env.conf.get("use_directory_urls", True)

    FRONTMATTER_RE = re.compile(r"(?s)\A---\s*\n(.*?)\n---\s*\n")
    TITLE_RE = re.compile(r"(?m)^\s*title\s*:\s*(.+?)\s*$")

    def human_title(stem: str) -> str:
        return re.sub(r"[-_]+", " ", stem).strip().title()

    def strip_quotes(s: str) -> str:
        s = s.strip()
        if (s.startswith('"') and s.endswith('"')) or (s.startswith("'") and s.endswith("'")):
            return s[1:-1].strip()
        return s

    def extract_title(md_text: str, fallback_stem: str) -> str:
        # 1) frontmatter title
        m = FRONTMATTER_RE.search(md_text)
        if m:
            fm = m.group(1)
            t = TITLE_RE.search(fm)
            if t:
                return strip_quotes(t.group(1))

        # 2) first H1
        for line in md_text.splitlines():
            if line.startswith("# "):
                return line[2:].strip()

        # 3) filename
        return human_title(fallback_stem)

    def md_to_link(rel_md_path: Path) -> str:
        rel = rel_md_path.as_posix().replace("\\", "/")

        # index/readme -> folder root
        if rel.lower() in ("index.md", "readme.md"):
            rel = ""

        if rel.endswith(".md"):
            rel = rel[:-3]

        if use_dir_urls:
            return (rel.rstrip("/") + "/") if rel else "./"
        else:
            return (rel + ".md") if rel else "index.md"

    def current_page_dir() -> Path | None:
        page = getattr(env, "page", None)
        src_path = getattr(getattr(page, "file", None), "src_path", None) if page else None
        if not src_path:
            return None
        return (docs_dir / src_path).resolve().parent

    def build_tree(md_files: list[Path], base: Path) -> dict:
        root = {"files": [], "dirs": {}}

        for p in md_files:
            rel = p.relative_to(base)
            parts = rel.parts
            if len(parts) == 1:
                root["files"].append(p)
                continue

            node = root
            for d in parts[:-1]:
                node = node["dirs"].setdefault(d, {"files": [], "dirs": {}})
            node["files"].append(p)

        # sort deterministically
        def sort_node(node):
            node["files"].sort(key=lambda p: p.as_posix().lower())
            for k in sorted(list(node["dirs"].keys()), key=lambda s: s.lower()):
                sort_node(node["dirs"][k])

        sort_node(root)
        return root

    def render_tree(node: dict, base: Path, indent: int = 0, rel_prefix: Path = Path(".")) -> list[str]:
        lines: list[str] = []

        # files at this level
        for p in node["files"]:
            rel_from_base = p.relative_to(base)
            title = None
            try:
                text = p.read_text(encoding="utf-8")
                title = extract_title(text, p.stem)
            except Exception:
                title = human_title(p.stem)

            url = md_to_link(rel_from_base)
            lines.append(f"{'  '*indent}- [{title}]({url})")

        # subdirs
        for dirname, child in node["dirs"].items():
            folder_title = human_title(dirname)
            lines.append(f"{'  '*indent}- **{folder_title}**")
            lines.extend(render_tree(child, base, indent=indent + 1))

        return lines

    @env.macro
    def list_pages(
        folder: str,
        recursive: bool = False,
        exclude: str = r"^(index|README)$",
    ):
        base = (docs_dir / folder).resolve()
        if not base.is_dir():
            return f"> **Error:** folder `{folder}` not found under `{env.conf['docs_dir']}`."

        # macro must be used from within that folder
        page_dir = current_page_dir()
        if page_dir is None:
            return (
                f"> **Error:** cannot detect current page path; `list_pages('{folder}')` "
                f"must be used from a page inside `{folder}/`."
            )

        try:
            page_dir.relative_to(base)
        except Exception:
            try:
                where = page_dir.relative_to(docs_dir).as_posix()
            except Exception:
                where = str(page_dir)
            return (
                f"> **Error:** `list_pages('{folder}')` must be used from a page inside `{folder}/`.\n"
                f"> Current page is in `{where}/`."
            )

        pattern = "**/*.md" if recursive else "*.md"
        ex = re.compile(exclude)

        md_files = sorted(
            (p for p in base.glob(pattern) if p.is_file() and not ex.search(p.stem)),
            key=lambda p: p.as_posix().lower(),
        )

        if not md_files:
            return "> *(No pages found)*"

        if not recursive:
            lines = []
            for p in md_files:
                try:
                    text = p.read_text(encoding="utf-8")
                    title = extract_title(text, p.stem)
                except Exception:
                    title = human_title(p.stem)

                rel_from_base = p.relative_to(base)
                url = md_to_link(rel_from_base)
                lines.append(f"- [{title}]({url})")
            return "\n".join(lines)

        tree = build_tree(md_files, base)
        lines = render_tree(tree, base)

        return "\n".join(lines)
