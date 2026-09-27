#!/usr/bin/env python3
"""buff_config.hpp 结构一致性守卫测试。

为什么需要它: 本机 (开发用的 Windows) 没有 yaml-cpp, 无法编译 C++ 单测,
所以用 Python 对 buff_config.hpp 做结构化自检, 保证:

  1. 大括号/圆括号/方括号配平
  2. loadBuffConfig 里 read("<key>", "<legacy>", c.<field>) 的 key 恰好是
     "<section>.<field>" (抓 key 与字段名不一致的低级错误)
  3. 每个 c.<field> 引用都是该节结构体的真实成员 (抓拼写错误)
  4. 每个字段都被 read (不存在只声明不读取的死字段)
  5. 每个字段都被 buffConfigToLines 打印 (调参时能看到实际生效值)
  6. 每个数值字段都有取值域校验 (require*/clamp*)
  7. detail::X 都是已定义的辅助函数

运行:
    python3 src/rm_tracker/test/test_buff_config_consistency.py
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
HEADER = REPO_ROOT / "src" / "rm_tracker" / "include" / "buff_config.hpp"
TEST_SOURCE = REPO_ROOT / "src" / "rm_tracker" / "test" / "buff_config_test.cpp"
CMAKE = REPO_ROOT / "src" / "rm_tracker" / "CMakeLists.txt"
YAML_FILE = REPO_ROOT / "src" / "rm_tracker" / "config" / "rm_fu.yaml"

SECTIONS = {
    "sync": "SyncConfig",
    "timing": "TimingConfig",
    "control": "ControlConfig",
    "ballistic": "BallisticConfig",
    "model": "ModelConfig",
    "recognition": "RecognitionConfig",
    "geometry": "GeometryConfig",
    "angle": "AngleConfig",
    "legacy_predictor": "LegacyPredictorConfig",
    "big_ransac": "BigRansacConfig",
    "world_tracker": "WorldTrackerConfig",
}

DUMP_FUNCTION = "buffConfigToLines"
VALIDATORS = ("requireFinite", "requirePositive", "requireAtLeast", "requireAtMost",
              "clampTo", "clampMin", "clampMax")
# 无需逐字段取值域校验的类型: 字符串 / 向量 / bool / 随机种子
NO_VALIDATOR_TYPES = {"bool", "std::string", "std::vector<double>", "unsigned int"}
MEMBER = re.compile(
    r"^\s*(std::size_t|std::string|double|int|bool|unsigned int|"
    r"std::vector<double>)\s+(\w+)\s*[={]", re.M)


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


class BuffConfigConsistencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw = HEADER.read_text(encoding="utf-8")
        cls.text = strip_comments(cls.raw)

    def test_header_and_test_are_registered(self):
        self.assertTrue(HEADER.is_file())
        self.assertTrue(TEST_SOURCE.is_file())
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("add_executable(buff_config_test", cmake)
        self.assertIn("add_test(NAME buff_config_test", cmake)
        # 纯 C++ 测试只依赖 yaml-cpp, 不应链接 catkin
        self.assertNotIn("target_link_libraries(buff_config_test\n    ${catkin_LIBRARIES}",
                         cmake)

    def test_brackets_are_balanced(self):
        text = re.sub(r'"(?:[^"\\]|\\.)*"', '""', self.text)
        for open_ch, close_ch, name in (("{", "}", "brace"), ("(", ")", "paren"),
                                        ("[", "]", "bracket")):
            depth = 0
            for ch in text:
                if ch == open_ch:
                    depth += 1
                elif ch == close_ch:
                    depth -= 1
                    self.assertGreaterEqual(depth, 0, f"{name} unbalanced")
            self.assertEqual(depth, 0, f"{name} unbalanced at EOF")

    def parsed_structs(self) -> dict[str, dict[str, str]]:
        structs = {}
        pattern = re.compile(r"struct\s+(\w+Config)\s*\{(.*?)\n\};", re.S)
        for name, body in pattern.findall(self.text):
            structs[name] = {field: ftype for ftype, field in MEMBER.findall(body)}
        return structs

    def test_all_sections_exist(self):
        structs = self.parsed_structs()
        for section, struct_name in SECTIONS.items():
            self.assertIn(struct_name, structs, f"{section}: missing {struct_name}")
            self.assertTrue(structs[struct_name], f"{struct_name} has no fields")

    def test_loader_keys_fields_and_validators_are_consistent(self):
        structs = self.parsed_structs()
        loader_start = self.text.index("loadBuffConfig(")
        loader_end = self.text.index(DUMP_FUNCTION)
        loader = self.text[loader_start:loader_end]

        for section, struct_name in SECTIONS.items():
            fields = structs[struct_name]
            with self.subTest(section=section):
                alias = re.search(rf"{struct_name}&\s+c\s*=\s*out\.{section};", loader)
                self.assertIsNotNone(
                    alias, f"{section}: no '{struct_name}& c = out.{section};'")

                block_start = alias.end()
                next_alias = re.search(r"\w+Config&\s+c\s*=\s*out\.",
                                       loader[block_start:])
                block = (loader[block_start:block_start + next_alias.start()]
                         if next_alias else loader[block_start:])

                # key 必须等于 "<section>.<field>"
                read_targets = set()
                for key, _legacy, field in re.findall(
                        r'read\(\s*"([^"]+)"\s*,\s*"([^"]*)"\s*,\s*c\.(\w+)', block):
                    read_targets.add(field)
                    self.assertEqual(
                        key, f"{section}.{field}",
                        f"read key '{key}' does not match '{section}.{field}'")

                # 引用必须是真实成员
                referenced = set(re.findall(r"c\.(\w+)", block))
                for field in sorted(referenced):
                    self.assertIn(field, fields,
                                  f"c.{field} is not a member of {struct_name}")

                # 每个字段都要被 read, 也要有校验 (bool/字符串/向量/种子除外)
                for field, ftype in sorted(fields.items()):
                    self.assertIn(field, referenced,
                                  f"{section}.{field} is never used")
                    self.assertIn(field, read_targets,
                                  f"{section}.{field} is never read from YAML")
                    if ftype in NO_VALIDATOR_TYPES:
                        continue
                    pattern = (r"(" + "|".join(VALIDATORS) + r")\(" +
                               re.escape("c." + field) + r"\b")
                    self.assertRegex(
                        block, pattern,
                        f"{section}.{field} has no range check")

    def test_every_field_is_dumped(self):
        structs = self.parsed_structs()
        dump = self.text[self.text.index(DUMP_FUNCTION):]
        for section, struct_name in SECTIONS.items():
            for field in sorted(structs[struct_name]):
                with self.subTest(section=section, field=field):
                    self.assertIn(f'"{section}.{field}"', dump,
                                  f"{section}.{field} is not printed by "
                                  f"{DUMP_FUNCTION}")

    def test_detail_helpers_are_defined(self):
        defined = set(re.findall(
            r"inline\s+(?:void|std::string|YAML::Node|template[^\n]*)\s*(\w+)\s*\(",
            self.text))
        defined |= {"lookup", "parseOr", "formatNumber"}
        for helper in sorted(set(re.findall(r"detail::(\w+)", self.text)) - defined):
            self.fail(f"detail::{helper} is used but not defined")

    def test_header_does_not_depend_on_ros(self):
        # 加载器不调用 ROS API, 才能脱离 catkin 单测。
        # 只看代码, 注释里提到 ROS_INFO 属于文档, 不算依赖。
        for token in ("ros/ros.h", "ROS_INFO", "ROS_WARN", "ROS_FATAL",
                      "ros::NodeHandle", "nh.param"):
            self.assertNotIn(token, self.text,
                             f"buff_config.hpp must stay ROS-free (found {token})")

    def test_yaml_keys_align_with_loader(self):
        """rm_fu.yaml 的 Buff 节键名必须与 loadBuffConfig 的 read() 完全对齐。

        打错一个键名 = 该参数被静默忽略、悄悄用默认值; 这是最隐蔽的调参陷阱。
        本测试用正则(不依赖 PyYAML)校验: yaml 里的每个键都是 loader 会读的键,
        且 loader 读的每个键都在 yaml 里(除非有过渡期旧扁平键)。
        """
        # loader 读的键: read("<key>", "<legacy>", ...)
        expected = {}
        for key, legacy in re.findall(
                r'read\(\s*"([^"]+)"\s*,\s*"([^"]*)"\s*,', self.text):
            expected[f"Buff.{key}"] = legacy

        raw = YAML_FILE.read_text(encoding="utf-8")
        actual = {}       # Buff.dotted -> value text
        top_level = set()  # 0 缩进旧扁平键
        dups = []
        section = None
        in_buff = False
        for line in raw.splitlines():
            stripped = line.rstrip()
            if not stripped.strip() or stripped.lstrip().startswith("#"):
                continue
            if re.match(r"^Buff:\s*(#.*)?$", stripped):
                in_buff = True
                continue
            if in_buff and re.match(r"^\S", stripped):
                in_buff = False
            if in_buff:
                m_sec = re.match(r"^  (\w+):\s*(#.*)?$", stripped)
                if m_sec:
                    section = m_sec.group(1)
                    continue
                m_field = re.match(r"^    ([\w.-]+):\s*(.*)$", stripped)
                if m_field and section:
                    key = f"Buff.{section}.{m_field.group(1)}"
                    if key in actual:
                        dups.append(key)
                    actual[key] = m_field.group(2)
                continue
            m_top = re.match(r"^(\w[\w.-]*):\s*(.*)$", stripped)
            if m_top:
                top_level.add(m_top.group(1))

        for key in sorted(expected):
            if key not in actual:
                legacy = expected[key]
                if legacy and legacy in top_level:
                    continue
                self.fail(f"rm_fu.yaml 缺少键: {key}")

        for key in sorted(actual):
            self.assertIn(key, expected,
                          f"rm_fu.yaml 出现未知键(会被静默忽略): {key}")

        for key in sorted(dups):
            self.fail(f"rm_fu.yaml 重复键(后者覆盖前者): {key}")


if __name__ == "__main__":
    unittest.main()
