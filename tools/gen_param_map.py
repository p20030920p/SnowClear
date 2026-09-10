#!/usr/bin/env python3
"""Regenerate the parameter maps from the loaders that read them.

`SystemConfig::to_map()` and `AblationSwitches::to_map()` must list exactly the
keys their `load()` functions read — no more, no fewer. Keeping that in sync by
hand is the kind of thing that silently rots, so the maps are generated from the
loader source instead:

    python3 tools/gen_param_map.py            # rewrite in place
    python3 tools/gen_param_map.py --check    # exit 1 if regeneration would change anything

`--check` is what CI should run: it turns "somebody added a parameter and forgot
to report it" into a build failure rather than a mystery at runtime.

Run from the repository root.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ABLATION_HPP = ROOT / "src/snowclear_core/include/snowclear/ablation_switches.hpp"
SYSTEM_CONFIG_CPP = ROOT / "src/snowclear_core/src/system_config.cpp"

# `member = params.get_<type>("key", default);`
ENTRY = re.compile(r'^\s*([\w.]+)\s*=\s*params\.get_(\w+)\("([^"]+)"', re.M)

# core accessor -> registry type
TYPE_OF = {"bool": "kBool", "int": "kInt", "float": "kDouble",
           "double": "kDouble", "string": "kString"}

HEADER = ("    // 参数名 -> 当前值。键集由 load() 生成（tools/gen_param_map.py），\n"
          "    // 保证\"代码真正读取的键\"与\"对外报告的键\"永不脱节；\n"
          "    // param_check 依赖它比对发布配置。\n")

IMPL_HEADER = """
// =====================================================================
// 参数名 -> 当前值
// ---------------------------------------------------------------------
// 键集由 load() 生成（tools/gen_param_map.py），因此"代码读取的键"与
// "这里报告的键"不可能不一致。param_check 用它把发布用的 YAML 配置与
// 编译期默认值逐项比对。
// =====================================================================
"""


def entries(text: str) -> list[tuple[str, str]]:
    """[(key, member_expression)] in source order."""
    return [(m.group(3), m.group(1)) for m in ENTRY.finditer(text)]


def typed_entries(text: str) -> list[tuple[str, str]]:
    """[(key, ParamType)] in source order, de-duplicated by key."""
    out: list[tuple[str, str]] = []
    seen: set[str] = set()
    for m in ENTRY.finditer(text):
        key, acc = m.group(3), m.group(2)
        if key in seen:
            continue
        seen.add(key)
        out.append((key, TYPE_OF[acc]))
    return out


def render_struct_map(rows: list[tuple[str, str]]) -> str:
    body = "\n".join(f'            {{"{k}", param_value_string({expr})}},' for k, expr in rows)
    return HEADER + ("    std::map<std::string, std::string> to_map() const {\n"
                     "        return {\n" + body + "\n        };\n    }\n")


def render_registry(rows: list[tuple[str, str]]) -> str:
    body = "\n".join(f'        {{"{k}", ParamType::{t}}},' for k, t in rows)
    return ("""
// =====================================================================
// 参数登记表
// ---------------------------------------------------------------------
// 核心读取的每一个参数及其声明类型。由 load() 生成
// （tools/gen_param_map.py），ROS 2 节点据此 declare_parameter，
// 因此 ros2 param 看到的就是算法真正读取的全部键。
// =====================================================================
const std::vector<ParamSpec>& param_registry() {
    static const std::vector<ParamSpec> kRegistry = {
""" + body + """
    };
    return kRegistry;
}
""".lstrip("\n"))


def render_class_map(rows: list[tuple[str, str]]) -> str:
    body = "\n".join(f'        {{"{k}", param_value_string({expr})}},' for k, expr in rows)
    return IMPL_HEADER + ("std::map<std::string, std::string> SystemConfig::to_map() const {\n"
                          "    return {\n" + body + "\n    };\n}\n")


def replace_block(text: str, start_marker: str, end_marker: str, new: str) -> str:
    start = text.index(start_marker)
    end = text.index(end_marker, start)
    return text[:start] + new + text[end:]


def regenerate() -> dict[pathlib.Path, str]:
    hpp = ABLATION_HPP.read_text(encoding="utf-8")
    load_start = hpp.index("void load(const ParamSource& params) {")
    load_end = hpp.index("    // 参数名 -> 当前值", load_start)
    hpp_new = replace_block(
        hpp,
        "    // 参数名 -> 当前值",
        "    // 打印开关状态",
        render_struct_map(entries(hpp[load_start:load_end])),
    )

    cpp = SYSTEM_CONFIG_CPP.read_text(encoding="utf-8")
    load_start = cpp.index("void SystemConfig::load(")
    load_end = cpp.index("void SystemConfig::validate()", load_start)
    cpp_new = replace_block(
        cpp,
        "\n// =====================================================================\n// 参数名 -> 当前值",
        "void SystemConfig::validate()",
        render_class_map(entries(cpp[load_start:load_end])),
    )

    # registry: both loaders, switches first (they are declared as bool)
    hpp_load_end = hpp.index("    // 参数名 -> 当前值", load_start)
    registry_rows = typed_entries(hpp[load_start:hpp_load_end]) + typed_entries(cpp[load_start:load_end])
    registry_marker = "\n// =====================================================================\n// 参数登记表"
    if registry_marker in cpp_new:
        cpp_new = replace_block(cpp_new, registry_marker,
                                "void SystemConfig::validate()",
                                render_registry(registry_rows) + "\n")
    else:  # first generation: insert before validate()
        cpp_new = cpp_new.replace("void SystemConfig::validate()",
                                  render_registry(registry_rows) + "\nvoid SystemConfig::validate()", 1)

    return {ABLATION_HPP: hpp_new, SYSTEM_CONFIG_CPP: cpp_new}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="do not write; fail if the generated maps are stale")
    args = ap.parse_args()

    stale = []
    for path, new in regenerate().items():
        old = path.read_text(encoding="utf-8")
        if old == new:
            print(f"  up to date  {path.relative_to(ROOT)}")
            continue
        stale.append(path)
        if args.check:
            print(f"  STALE       {path.relative_to(ROOT)}")
        else:
            path.write_text(new, encoding="utf-8")
            print(f"  regenerated {path.relative_to(ROOT)}")

    if args.check and stale:
        print("\n参数表已过期：运行 `python3 tools/gen_param_map.py` 重新生成。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
