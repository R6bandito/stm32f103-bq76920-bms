#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stack_analyzer.py -- ARM Cortex-M 编译期静态栈深度分析工具
================================================================
* 零第三方依赖，仅需 Python 3.6+ 标准库
* 同时支持 GCC (arm-none-eabi) 与 Keil MDK (ARMCC AC5 / ARMCLANG AC6)

数据源（按优先级合并）：
  帧大小：stack_config.json 覆盖 > GCC .su > Keil armlink 调用图 .htm > 反汇编序言推算
  调用图：反汇编 (objdump / fromelf) 中的 BL/BLX/B(尾调用) + Keil .htm [Calls] + 配置补全

典型用法：
  # Keil (本工程)
  python tools/stack_analyzer.py --build-dir . --entry Reset_Handler \
         --thread main_thread_entry --thread rt_thread_idle_entry --auto-isr
  # GCC
  python tools/stack_analyzer.py --build-dir build --elf build/app.elf --entry main --limit 2048

退出码：0 = 通过；1 = 超过门限 (或 --strict 下存在无界/未解析项)；2 = 参数/环境错误
"""
import argparse
import bisect
import difflib
import fnmatch
import glob
import html
import json
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

VERSION = "1.0.0"

HW_FRAME_BASIC = 32      # R0-R3, R12, LR, PC, xPSR
HW_FRAME_FPU = 104       # + S0-S15, FPSCR, reserved
ALIGN_PAD = 4            # CCR.STKALIGN 导致的最多 4 字节对齐填充

# ----------------------------------------------------------------------------
# 终端样式
# ----------------------------------------------------------------------------


class Style:
    color = False
    ascii = False


def _c(text, code):
    return "\033[%sm%s\033[0m" % (code, text) if Style.color else str(text)


def red(t):
    return _c(t, "31;1")


def yellow(t):
    return _c(t, "33;1")


def green(t):
    return _c(t, "32;1")


def cyan(t):
    return _c(t, "36")


def bold(t):
    return _c(t, "1")


def dim(t):
    return _c(t, "2")


def tree_glyphs():
    if Style.ascii:
        return "`-- ", "|-- ", "|   ", "    "
    return "└── ", "├── ", "│   ", "    "


def die(msg, code=2):
    sys.stderr.write("错误: %s\n" % msg)
    sys.exit(code)


def parse_int(s):
    s = s.strip().lstrip("#").strip()
    neg = s.startswith("-")
    s = s.lstrip("+-").strip()
    v = int(s, 16) if s.lower().startswith("0x") else int(s, 10)
    return -v if neg else v


# ----------------------------------------------------------------------------
# 指令模型 & 通用解析工具
# ----------------------------------------------------------------------------

COND = r"(?:eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al)"
BRANCH_RE = re.compile(r"^b%s?$" % COND)       # b / beq / bls / ble ...
CALL_RE = re.compile(r"^bl%s?$" % COND)        # bl / bleq (IT 块)
BLX_RE = re.compile(r"^blx%s?$" % COND)
BX_RE = re.compile(r"^bx%s?$" % COND)
MAPPING_SYM_RE = re.compile(r"^\$[adtp](\..*)?$")          # $d / $t / $d.realdata
SECTION_LABEL_RE = re.compile(r"^(i\.|\.|!!|x\$|RESET$)")   # armlink 输入段名
BOUNDARY_SYM_RE = re.compile(r"(_End|_end|_Limit|_Base|_Size|\$\$\w+)$|^__Vectors")
REG_ALIASES = {"sb": "r9", "sl": "r10", "fp": "r11", "ip": "r12",
               "sp": "r13", "lr": "r14", "pc": "r15"}
REG_RE = re.compile(r"^(r\d{1,2}|sb|sl|fp|ip|sp|lr|pc)$")


class Insn(object):
    __slots__ = ("addr", "mnem", "ops", "comment", "target", "lit_addr", "lit_value", "text")

    def __init__(self, addr, mnem, ops, comment, text):
        self.addr = addr
        m = mnem.lower()
        for suf in (".w", ".n"):
            if m.endswith(suf):
                m = m[:-2]
        self.mnem = m
        self.ops = ops.strip().lower()
        self.comment = comment.strip()
        self.text = text.strip()
        self.target = None      # 直接跳转目标地址
        self.lit_addr = None    # PC 相对加载的文字池地址
        self.lit_value = None   # 文字池中的值（若已知）


class RawFunc(object):
    def __init__(self, names, addr):
        self.names = list(names)
        self.addr = addr
        self.insns = []


def split_ops(ops):
    """按逗号切分操作数（忽略 {} [] 内的逗号）"""
    out, depth, cur = [], 0, ""
    for ch in ops:
        if ch in "{[":
            depth += 1
        elif ch in "}]":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def norm_reg(r):
    r = r.strip().lower().rstrip("!")
    return REG_ALIASES.get(r, r)


def is_reg(tok):
    return bool(REG_RE.match(tok.strip().lower().rstrip("!")))


def parse_reglist(text):
    m = re.search(r"\{([^}]*)\}", text)
    if not m:
        return []
    out = []
    for part in m.group(1).split(","):
        part = part.strip().lower()
        if not part:
            continue
        rng = part.split("-")
        if len(rng) == 2:
            a, b = norm_reg(rng[0]), norm_reg(rng[1])
            ma, mb = re.match(r"([rsd])(\d+)$", a), re.match(r"([rsd])(\d+)$", b)
            if ma and mb and ma.group(1) == mb.group(1):
                out += ["%s%d" % (ma.group(1), i) for i in range(int(ma.group(2)), int(mb.group(2)) + 1)]
        else:
            out.append(norm_reg(part))
    return out


def reglist_bytes(regs):
    return sum(8 if r.startswith("d") else 4 for r in regs)


# ----------------------------------------------------------------------------
# 反汇编解析：GNU objdump
# ----------------------------------------------------------------------------

OBJ_FUNC_RE = re.compile(r"^([0-9a-fA-F]+)\s+<([^>]+)>:\s*$")
OBJ_INS_RE = re.compile(r"^\s*([0-9a-fA-F]+):\t([^\t]*)\t([^\t]*)\t?(.*)$")


def parse_objdump(text):
    funcs, words, cur = [], {}, None
    for line in text.splitlines():
        m = OBJ_FUNC_RE.match(line)
        if m:
            cur = RawFunc([m.group(2)], int(m.group(1), 16))
            funcs.append(cur)
            continue
        if line.startswith("Disassembly of section"):
            cur = None
            continue
        m = OBJ_INS_RE.match(line)
        if not m or cur is None:
            continue
        addr = int(m.group(1), 16)
        mnem = m.group(3).strip()
        rest = m.group(4).replace("\t", " ")
        if not mnem:
            continue
        if mnem.startswith("."):
            if mnem == ".word":
                try:
                    words[addr] = parse_int(rest.split()[0]) & 0xFFFFFFFF
                except (ValueError, IndexError):
                    pass
            continue
        ops, comment = rest, ""
        for sep in (";", "@"):
            if sep in ops:
                ops, _, comment = ops.partition(sep)
                break
        ins = Insn(addr, mnem, ops, comment, "%s %s" % (mnem, rest.strip()))
        mt = re.match(r"^([0-9a-fA-F]+)\s*<", ins.ops)
        if mt:
            ins.target = int(mt.group(1), 16)
        if ins.mnem.startswith("ldr") and "[pc" in ins.ops:
            ml = re.search(r"\(?([0-9a-fA-F]+)\s*<", comment)
            if ml:
                ins.lit_addr = int(ml.group(1), 16)
        cur.insns.append(ins)
    for f in funcs:
        for ins in f.insns:
            if ins.lit_addr is not None:
                ins.lit_value = words.get(ins.lit_addr)
    return funcs, words


# ----------------------------------------------------------------------------
# 反汇编解析：Keil fromelf --text -c
# ----------------------------------------------------------------------------

FE_INS_RE = re.compile(r"^\s+0x([0-9a-fA-F]{8}):\s{4}")


def parse_fromelf(text):
    funcs, words = [], {}
    in_exec, cur, pending = False, None, []
    for line in text.splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("** Section"):
            in_exec = "SHF_EXECINSTR" in s
            cur, pending = None, []
            continue
        if not in_exec:
            continue
        m = FE_INS_RE.match(line)
        if m:
            addr = int(m.group(1), 16)
            st = m.end()
            hexfield = line[st:st + 12].strip()
            rest = line[st + 20:].strip()
            if not re.match(r"^[0-9a-fA-F]+$", hexfield) or not re.match(r"^[A-Z]", rest):
                # 列宽不符时的兜底：跳过 hex + ascii，找第一个大写助记符
                mm = re.match(r"^\s+0x[0-9a-fA-F]{8}:\s+[0-9a-fA-F]+\s+.{1,4}?\s+([A-Z][A-Z0-9.]*\b.*)$", line)
                if not mm:
                    continue
                rest = mm.group(1).strip()
            parts = rest.split(None, 1)
            mnem = parts[0]
            opsfull = parts[1] if len(parts) > 1 else ""
            ops, _, comment = opsfull.partition(";")
            if mnem.upper() in ("DCD", "DCW", "DCB"):
                if mnem.upper() == "DCD":
                    try:
                        words[addr] = parse_int(ops) & 0xFFFFFFFF
                    except ValueError:
                        pass
                pending = []          # 数据标签，丢弃
                continue
            if pending:
                cur = RawFunc(pending, addr)
                funcs.append(cur)
                pending = []
            if cur is None:
                continue
            ins = Insn(addr, mnem, ops, comment, rest)
            mt = re.search(r"0x([0-9a-fA-F]+)", ins.ops) or re.search(r"0x([0-9a-fA-F]+)", comment)
            if mt and (BRANCH_RE.match(ins.mnem) or CALL_RE.match(ins.mnem) or BLX_RE.match(ins.mnem)):
                ins.target = int(mt.group(1), 16)
            if ins.mnem.startswith("ldr") and "[pc" in ins.ops:
                ml = re.search(r"\[0x([0-9a-fA-F]+)\]\s*=\s*0x([0-9a-fA-F]+)", comment)
                if ml:
                    ins.lit_addr = int(ml.group(1), 16)
                    ins.lit_value = int(ml.group(2), 16)
            cur.insns.append(ins)
            continue
        # 标签行：缩进 + 单个不含空格的名字
        if line[:1] == " " and " " not in s and not s.startswith("0x"):
            if MAPPING_SYM_RE.match(s):
                continue
            if SECTION_LABEL_RE.match(s):
                # 段标签：之前挂起的标签是数据标签（如 __Vectors_End）。
                # 当前函数不结束 —— armlink 的 .ARM.Collect$$ 片段会跨多个段连续执行。
                pending = []
                continue
            pending.append(s)
    return funcs, words


def detect_disasm_format(text):
    head = text[:20000]
    if "** Section #" in head or "ELF Header Information" in head:
        return "fromelf"
    return "objdump"


# ----------------------------------------------------------------------------
# 单函数分析：栈帧推算 + 调用边 + 间接调用
# ----------------------------------------------------------------------------

DYNAMIC = "dynamic"


def stack_effect(mn, ops, opsl, regs):
    """返回 (delta, is_return)。delta>0 分配，<0 释放，DYNAMIC 表示运行时决定"""
    first = ops[0] if ops else ""
    if mn == "push" or (mn in ("stmdb", "stmfd") and first in ("sp!", "r13!")):
        return reglist_bytes(parse_reglist(opsl)), False
    if mn == "vpush" or (mn in ("vstmdb", "fstmdbd", "fstmdbs") and first in ("sp!", "r13!")):
        return reglist_bytes(parse_reglist(opsl)), False
    if mn == "pop" or (mn in ("ldm", "ldmia", "ldmfd") and first in ("sp!", "r13!")):
        rl = parse_reglist(opsl)
        return -reglist_bytes(rl), ("r15" in rl)
    if mn == "vpop" or (mn in ("vldm", "vldmia") and first in ("sp!", "r13!")):
        return -reglist_bytes(parse_reglist(opsl)), False
    if mn in ("sub", "subw", "subs") and first == "sp" and len(ops) >= 2:
        last = ops[-1]
        if last.startswith("#"):
            return parse_int(last), False
        r = norm_reg(last)
        if r in regs and r != "r13":
            return regs[r], False          # movw ip,#N ; sub sp, ip
        return DYNAMIC, False              # VLA / alloca
    if mn in ("add", "addw", "adds") and first == "sp" and len(ops) >= 2:
        last = ops[-1]
        if last.startswith("#"):
            return -parse_int(last), False
        return 0, False
    if mn in ("str", "strd") and "[sp" in opsl and opsl.endswith("!"):
        mm = re.search(r"#(-?(?:0x[0-9a-f]+|\d+))\]!", opsl)
        if mm:
            v = parse_int(mm.group(1))
            return (-v if v < 0 else 0), False
    if mn == "ldr" and first in ("pc", "r15") and "[sp]" in opsl:
        mm = re.search(r"\],\s*#(0x[0-9a-f]+|\d+)", opsl)
        return (-parse_int(mm.group(1)) if mm else 0), True
    if BX_RE.match(mn) and norm_reg(first) == "r14":
        return 0, True
    return 0, False


NO_DEST_PREFIX = ("str", "stm", "vst", "push", "cmp", "cmn", "tst", "teq", "cb", "it",
                  "nop", "dmb", "dsb", "isb", "svc", "bkpt", "cps", "msr", "pld", "wfi", "wfe", "sev")


def update_regs(regs, mn, ops, ins):
    if not ops:
        return
    rd = norm_reg(ops[0])
    if mn.startswith("ldr") and len(ops) >= 2 and ops[1].startswith("[pc"):
        if ins.lit_value is not None:
            regs[rd] = ins.lit_value
        else:
            regs.pop(rd, None)
    elif mn == "movw" or (mn in ("mov", "movs") and len(ops) >= 2 and ops[1].startswith("#")):
        try:
            regs[rd] = parse_int(ops[1]) & 0xFFFFFFFF
        except ValueError:
            regs.pop(rd, None)
    elif mn == "movt":
        if rd in regs and len(ops) >= 2:
            try:
                regs[rd] = (regs[rd] & 0xFFFF) | ((parse_int(ops[1]) & 0xFFFF) << 16)
            except ValueError:
                regs.pop(rd, None)
    elif mn in ("mov", "movs") and len(ops) >= 2 and is_reg(ops[1]):
        src = norm_reg(ops[1])
        if src in regs:
            regs[rd] = regs[src]
        else:
            regs.pop(rd, None)
    elif mn in ("pop", "ldm", "ldmia", "ldmfd"):
        for r in parse_reglist(ins.ops):
            regs.pop(r, None)
    elif BRANCH_RE.match(mn) or BX_RE.match(mn) or mn.startswith(NO_DEST_PREFIX):
        pass
    elif is_reg(rd):
        regs.pop(rd, None)


class Function(object):
    def __init__(self, name, addr=None):
        self.name = name
        self.addr = addr
        self.aliases = {name}
        self.frame = None
        self.frame_src = None          # su / htm / disasm / config
        self.alloc = None              # static / bounded / dynamic
        self.su_frame = None
        self.su_alloc = None
        self.su_files = set()
        self.htm_frame = None
        self.htm_obj = None
        self.dis_frame = None
        self.dis_dynamic = False
        self.callees = {}              # callee -> kind (call / tail / const / htm / cfg)
        self.indirect = []             # [(addr, text)]
        self.cfg_resolved = False


class Program(object):
    def __init__(self):
        self.funcs = {}
        self.alias = {}
        self.starts = []
        self.start_names = []
        self.words = {}
        self.addr_taken = set()

    def add(self, name, addr=None, aliases=()):
        f = self.funcs.get(self.alias.get(name, name))
        if f is None:
            f = Function(name, addr)
            self.funcs[name] = f
        for a in [name] + list(aliases):
            f.aliases.add(a)
            self.alias.setdefault(a, f.name)
        return f

    def resolve(self, name):
        if name in self.alias:
            return self.funcs[self.alias[name]]
        m = re.match(r"^(.*)\.lto_priv\.\d+$", name)
        if m and m.group(1) in self.alias:
            return self.funcs[self.alias[m.group(1)]]
        return None

    def build_index(self):
        pairs = sorted((f.addr, f.name) for f in self.funcs.values() if f.addr is not None)
        self.starts = [p[0] for p in pairs]
        self.start_names = [p[1] for p in pairs]

    def at(self, addr, exact=False):
        addr &= ~1
        i = bisect.bisect_right(self.starts, addr) - 1
        if i < 0:
            return None
        if exact and self.starts[i] != addr:
            return None
        return self.funcs[self.start_names[i]]


def analyze_disasm(prog, rawfuncs):
    for rf in rawfuncs:
        names = [n for n in rf.names if not SECTION_LABEL_RE.match(n)] or rf.names
        # 同地址多个符号：边界符号（__Vectors_End / xxx$$Limit）不作为主名
        names.sort(key=lambda n: 1 if BOUNDARY_SYM_RE.search(n) else 0)
        f = prog.add(names[0], rf.addr, names[1:])
        f._insns = rf.insns
    prog.build_index()

    for f in list(prog.funcs.values()):
        insns = getattr(f, "_insns", None)
        if insns is None:
            continue
        local_targets = {i.target for i in insns if i.target is not None and BRANCH_RE.match(i.mnem)}
        off = maxoff = 0
        epi_start = None
        regs = {}
        for ins in insns:
            if ins.addr in local_targets:
                regs.clear()
            mn = ins.mnem
            ops = split_ops(ins.ops)
            first = norm_reg(ops[0]) if ops else ""

            # ---- 栈帧推算 ----
            delta, is_ret = stack_effect(mn, ops, ins.ops, regs)
            if delta == DYNAMIC:
                f.dis_dynamic = True
                delta = 0
            if delta > 0:
                off += delta
                maxoff = max(maxoff, off)
                epi_start = None
            elif delta < 0:
                if epi_start is None:
                    epi_start = off
                off = max(0, off + delta)
            elif not is_ret:
                epi_start = None
            if is_ret and epi_start is not None:
                off, epi_start = epi_start, None   # 后续代码仍处在原栈帧内（多出口函数）

            # ---- 调用 / 跳转 ----
            if CALL_RE.match(mn) or (BLX_RE.match(mn) and ins.target is not None and not is_reg(first)):
                callee = prog.at(ins.target) if ins.target is not None else None
                if callee is not None and callee is not f:
                    f.callees.setdefault(callee.name, "call")
                for r in ("r0", "r1", "r2", "r3", "r12", "r14"):
                    regs.pop(r, None)
            elif BLX_RE.match(mn):
                val = regs.get(first)
                callee = prog.at(val, exact=True) if val is not None else None
                if callee is not None:
                    f.callees.setdefault(callee.name, "const")
                else:
                    f.indirect.append((ins.addr, ins.text))
                for r in ("r0", "r1", "r2", "r3", "r12", "r14"):
                    regs.pop(r, None)
            elif BRANCH_RE.match(mn) and ins.target is not None:
                callee = prog.at(ins.target, exact=True)
                if callee is not None and callee is not f:
                    f.callees.setdefault(callee.name, "tail")
            elif BX_RE.match(mn) and first != "r14":
                val = regs.get(first)
                callee = prog.at(val, exact=True) if val is not None else None
                if callee is not None and callee is not f:
                    f.callees.setdefault(callee.name, "tail")
                elif callee is None:
                    f.indirect.append((ins.addr, ins.text))
            elif (mn in ("mov", "ldr") and first == "r15" and "[sp" not in ins.ops):
                f.indirect.append((ins.addr, ins.text))

            update_regs(regs, mn, ops, ins)
        f.dis_frame = maxoff

        # 顺序落入（fall-through）：armlink 的 .ARM.Collect$$ 片段、汇编函数等末尾没有跳转，
        # 直接执行到下一个符号。保守地按尾调用边处理。
        body = [i for i in insns if not (i.mnem in ("movs", "mov") and i.ops.replace(" ", "") == "r0,r0")
                and i.mnem != "nop"]
        if body:
            tail = body[-1]
            ops = split_ops(tail.ops)
            first = norm_reg(ops[0]) if ops else ""
            terminal = (tail.mnem == "b" or BX_RE.match(tail.mnem) or first == "r15"
                        or ((tail.mnem == "pop" or tail.mnem.startswith("ldm")) and "pc" in tail.ops))
            if not terminal:
                nxt = prog.at(tail.addr + 2, exact=True) or prog.at(tail.addr + 4, exact=True)
                if nxt is not None and nxt is not f:
                    f.callees.setdefault(nxt.name, "tail")
        del f._insns

    # 被取地址的函数（文字池/向量表中出现的函数入口）= 可能的函数指针目标
    for v in prog.words.values():
        g = prog.at(v, exact=True) if v & 1 else None   # Thumb 函数指针最低位为 1
        if g is not None:
            prog.addr_taken.add(g.name)


# ----------------------------------------------------------------------------
# 其它数据源：.su / Keil .htm / 配置
# ----------------------------------------------------------------------------

SU_RE = re.compile(r"^(.*?):(\d+):(\d+):(.+?)\t(\d+)\t(\S+)\s*$")


def parse_su_files(dirs):
    result, files = {}, []
    for d in dirs:
        for root, _, names in os.walk(d):
            for n in names:
                if n.endswith(".su"):
                    files.append(os.path.join(root, n))
    for path in sorted(set(files)):
        with open(path, encoding="utf-8-sig", errors="replace") as fh:
            for line in fh:
                m = SU_RE.match(line.rstrip("\n"))
                if not m:
                    continue
                name = m.group(4).strip()
                if "(" in name:                       # C++: "int foo(int)"
                    name = name.split("(")[0].split()[-1]
                size, q = int(m.group(5)), m.group(6)
                alloc = "bounded" if "bounded" in q else ("dynamic" if "dynamic" in q else "static")
                src = "%s:%s" % (m.group(1), m.group(2))
                old = result.get(name)
                if old is None or size > old[0]:
                    result[name] = (size, alloc, {src} | (old[2] if old else set()))
                else:
                    old[2].add(src)
    return result, files


def parse_keil_htm(text):
    entries, fptrs = {}, set()
    mx = re.search(r"Maximum Stack Usage =\s*(\d+) bytes", text)
    m = re.search(r"Function Pointers\s*</H3>(.*?)</UL>", text, re.S)
    if m:
        for nm in re.findall(r'<a href="#\[[0-9a-f]+\]">([^<]+)</a>', m.group(1)):
            fptrs.add(html.unescape(nm).strip())
    for chunk in re.split(r"<P><STRONG>", text)[1:]:
        mm = re.match(r'<a name="\[[0-9a-f]+\]"></a>(.*?)</STRONG>\s*\(([^\r\n]*)', chunk, re.S)
        if not mm:
            continue
        name = html.unescape(mm.group(1)).strip()
        header = mm.group(2)
        ms = re.search(r"Stack size (\d+|unknown) bytes", header)
        stack = int(ms.group(1)) if ms and ms.group(1).isdigit() else None
        mo = re.search(r",\s*([^,()]+?\.o)\(", header)
        calls = []
        mc = re.search(r"\[Calls\]<UL>(.*?)</UL>", chunk, re.S)
        if mc:
            calls = [html.unescape(x).strip() for x in re.findall(r"&nbsp;&nbsp;&nbsp;([^<\r\n]+)", mc.group(1))]
        entries[name] = (stack, calls, mo.group(1) if mo else None)
    return entries, fptrs, (int(mx.group(1)) if mx else None)


def load_config(path):
    if not path:
        return {}
    try:
        with open(path, encoding="utf-8-sig") as fh:
            return json.load(fh)
    except (OSError, ValueError) as e:
        die("无法读取配置文件 %s: %s" % (path, e))


# ----------------------------------------------------------------------------
# 工程 / 工具链探测
# ----------------------------------------------------------------------------


def _read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            return fh.read()
    except OSError:
        return ""


def detect_project(root):
    info = {"root": os.path.abspath(root), "kind": None, "toolchain": None, "elf": None,
            "htm": None, "fpu": None, "fromelf_dirs": [], "notes": [], "startup_stack": None}
    projs = glob.glob(os.path.join(root, "*.uvprojx")) + glob.glob(os.path.join(root, "*", "*.uvprojx"))
    if projs:
        p = projs[0]
        info["kind"] = "keil"
        try:
            tree = ET.parse(p).getroot()
            t = tree.find(".//Target")

            def tx(tag):
                e = t.find(".//" + tag) if t is not None else None
                return (e.text or "").strip() if e is not None else ""
            base = os.path.dirname(p)
            ac6 = tx("uAC6") == "1"
            info["toolchain"] = ("ARMCLANG (AC6) " if ac6 else "ARMCC (AC5) ") + tx("pCCUsed").split("::")[1:2][0] \
                if "::" in tx("pCCUsed") else ("ARMCLANG (AC6)" if ac6 else "ARMCC (AC5)")
            outdir = os.path.normpath(os.path.join(base, tx("OutputDirectory").replace("\\", os.sep)))
            name = tx("OutputName")
            cand = os.path.join(outdir, name + ".axf")
            if os.path.isfile(cand):
                info["elf"] = cand
            htm = os.path.join(outdir, name + ".htm")
            if os.path.isfile(htm):
                info["htm"] = htm
            cpu = tx("Cpu")
            vp = tx("RvdsVP")
            info["cpu"] = (re.search(r'CPUTYPE\("([^"]+)"\)', cpu) or re.search("(.*)", "?")).group(1)
            info["fpu"] = ("FPU" in cpu) and vp not in ("", "0", "1")
            for log in glob.glob(os.path.join(outdir, "*.build_log.htm")):
                mm = re.search(r"Toolchain Path:\s*(.+)", _read(log))
                if mm:
                    info["fromelf_dirs"].append(mm.group(1).strip())
            info["notes"].append("Keil 工程: %s" % os.path.basename(p))
            if ac6:
                info["notes"].append("AC6: 帧大小主要来自 armlink 调用图 (.htm) 与反汇编推算")
            else:
                info["notes"].append("AC5 不支持 -fstack-usage；帧大小来自 armlink 调用图 (.htm) 与反汇编推算")
            if not info["htm"]:
                info["notes"].append("未发现 %s.htm：请在 Options->Listing 勾选 'Callgraph' (armlink --callgraph)" % name)
        except (ET.ParseError, IndexError, AttributeError) as e:
            info["notes"].append("解析 uvprojx 失败: %s" % e)
    else:
        scripts = []
        for pat in ("Makefile", "makefile", "*.mk", "CMakeLists.txt", "*.cmake", "cmake/*.cmake",
                    "SConstruct", "SConscript", "rtconfig.py"):
            scripts += glob.glob(os.path.join(root, pat))
        blob = "\n".join(_read(s) for s in scripts)
        if scripts:
            info["kind"] = "gcc" if ("arm-none-eabi" in blob or "gcc" in blob.lower()) else "unknown"
            info["toolchain"] = "arm-none-eabi-gcc" if "arm-none-eabi" in blob else "未确定"
            info["notes"].append("构建脚本: %s" % ", ".join(os.path.basename(s) for s in scripts))
            if "-fstack-usage" not in blob:
                info["notes"].append("未发现 -fstack-usage：请在 CFLAGS 中添加 (CMake: add_compile_options(-fstack-usage))")
            if "-flto" in blob:
                info["notes"].append("检测到 -flto：编译期 .su 为 LTO 前数据；需在链接参数中同样加 -fstack-usage，"
                                     "并用 --su-dir 指向 *.ltrans*.su 所在目录")
    # 启动文件中定义的主栈大小
    for pat in ("**/startup*.s", "**/startup*.S", "**/*.ld", "**/*.sct"):
        for fpath in glob.glob(os.path.join(root, pat), recursive=True):
            txt = _read(fpath)
            mm = re.search(r"Stack_Size\s+EQU\s+(0x[0-9a-fA-F]+|\d+)", txt) or \
                re.search(r"_Min_Stack_Size\s*=\s*(0x[0-9a-fA-F]+|\d+)", txt)
            if mm:
                info["startup_stack"] = (parse_int(mm.group(1)), os.path.relpath(fpath, root))
                break
        if info["startup_stack"]:
            break
    return info


def find_elf(build_dir):
    c = []
    for ext in ("*.elf", "*.axf", "*.out"):
        c += glob.glob(os.path.join(build_dir, "**", ext), recursive=True)
    return max(c, key=os.path.getmtime) if c else None


def find_tool(names, extra_dirs):
    for n in names:
        p = shutil.which(n)
        if p:
            return p
    for d in extra_dirs:
        for n in names:
            for cand in (os.path.join(d, n), os.path.join(d, n + ".exe")):
                if os.path.isfile(cand):
                    return cand
    return None


def run_disassembler(args, info):
    if args.disasm:
        text = _read(args.disasm)
        if not text:
            die("无法读取反汇编文件 %s" % args.disasm)
        return text, detect_disasm_format(text), args.disasm
    elf = args.elf
    keil_dirs = info.get("fromelf_dirs", []) + [r"C:\Keil_v5\ARM\ARMCC\bin", r"C:\Keil_v5\ARM\ARMCLANG\bin",
                                                 r"D:\Keil_v5\ARM\ARMCC\bin", r"C:\Keil_v5\ARM\ARMCLANG\bin"]
    objdump = args.objdump or find_tool(["arm-none-eabi-objdump"], [])
    fromelf = args.fromelf or find_tool(["fromelf"], keil_dirs)
    order = [("objdump", objdump), ("fromelf", fromelf)]
    if info.get("kind") == "keil":
        order.reverse()
    for kind, exe in order:
        if not exe:
            continue
        cmd = [exe, "-d", "-w", elf] if kind == "objdump" else [exe, "--text", "-c", elf]
        try:
            out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
            return out.stdout.decode("utf-8", errors="replace"), kind, " ".join(cmd[:-1]) + " <elf>"
        except (OSError, subprocess.CalledProcessError) as e:
            sys.stderr.write("警告: %s 执行失败: %s\n" % (exe, e))
    return None, None, None


# ----------------------------------------------------------------------------
# 合并数据源
# ----------------------------------------------------------------------------


def build_program(args, info, cfg):
    prog = Program()
    sources = {"disasm": None, "htm": None, "su_files": 0, "htm_max": None}

    text, fmt, how = (None, None, None)
    if args.elf or args.disasm:
        text, fmt, how = run_disassembler(args, info)
    if text:
        raw, words = parse_fromelf(text) if fmt == "fromelf" else parse_objdump(text)
        prog.words = words
        analyze_disasm(prog, raw)
        sources["disasm"] = "%s (%d 个函数)" % (how, len(raw))

    # Keil 调用图
    htm_path = args.htm or info.get("htm")
    if htm_path and os.path.isfile(htm_path) and not args.no_htm:
        entries, fptrs, hmax = parse_keil_htm(_read(htm_path))
        sources["htm"] = "%s (%d 个符号)" % (os.path.relpath(htm_path), len(entries))
        sources["htm_max"] = hmax
        no_disasm = not prog.funcs
        for name, (stack, calls, obj) in entries.items():
            f = prog.resolve(name)
            if f is None:
                if not no_disasm:
                    continue
                f = prog.add(name)
            if stack is not None:
                f.htm_frame = stack if f.htm_frame is None else max(f.htm_frame, stack)
            f.htm_obj = f.htm_obj or obj
        for name, (stack, calls, obj) in entries.items():
            f = prog.resolve(name)
            if f is None:
                continue
            for c in calls:
                g = prog.resolve(c)
                if g is not None and g is not f:
                    f.callees.setdefault(g.name, "htm")
        for n in fptrs:
            g = prog.resolve(n)
            if g is not None:
                prog.addr_taken.add(g.name)

    # GCC .su
    su_dirs = [d for d in ([args.build_dir] + (args.su_dir or [])) if d and os.path.isdir(d)]
    su, su_files = parse_su_files(su_dirs)
    sources["su_files"] = len(su_files)
    for name, (size, alloc, srcs) in su.items():
        f = prog.resolve(name)
        if f is None:
            continue
        f.su_frame, f.su_alloc = size, alloc
        f.su_files |= srcs

    if not prog.funcs:
        die("没有可用的数据源：请用 --elf 指定 .elf/.axf（并确保 objdump/fromelf 可用），或提供 Keil .htm / --disasm")

    # 配置：帧覆盖
    overrides = cfg.get("frame_override", {})
    for name, size in overrides.items():
        f = prog.resolve(name)
        if f is None:
            sys.stderr.write("警告: frame_override 中的函数不存在: %s\n" % name)
            continue
        f.frame, f.frame_src = int(size), "config"

    for f in prog.funcs.values():
        if f.frame_src == "config":
            f.alloc = "static"
            continue
        if f.su_frame is not None:
            f.frame, f.frame_src, f.alloc = f.su_frame, "su", f.su_alloc
        elif f.htm_frame is not None:
            f.frame, f.frame_src = f.htm_frame, "htm"
            f.alloc = "dynamic" if f.dis_dynamic else "static"
        elif f.dis_frame is not None:
            f.frame, f.frame_src = f.dis_frame, "disasm"
            f.alloc = "dynamic" if f.dis_dynamic else "static"

    # 配置：函数指针补全
    for caller, callees in cfg.get("indirect_calls", {}).items():
        f = prog.resolve(caller)
        if f is None:
            sys.stderr.write("警告: indirect_calls 中的调用者不存在: %s\n" % caller)
            continue
        f.cfg_resolved = True
        for c in callees:
            g = prog.resolve(c)
            if g is None:
                sys.stderr.write("警告: indirect_calls 中的被调者不存在: %s\n" % c)
                continue
            f.callees[g.name] = "cfg"
    return prog, sources


# ----------------------------------------------------------------------------
# 调用图分析：三色 DFS + 最长路径
# ----------------------------------------------------------------------------

WHITE, GRAY, BLACK = 0, 1, 2


class EntryResult(object):
    def __init__(self, name, etype, overhead):
        self.name, self.etype, self.overhead = name, etype, overhead
        self.worst = 0
        self.best = {}
        self.nxt = {}
        self.cycles = []
        self.reachable = set()
        self.limit = None

    @property
    def total(self):
        return self.worst + self.overhead

    def path(self):
        out, n, seen = [], self.name, set()
        while n is not None and n not in seen:
            seen.add(n)
            out.append(n)
            n = self.nxt.get(n)
        return out


def analyze_entry(prog, name, etype, overhead, ignore):
    res = EntryResult(name, etype, overhead)
    color, stack, best, nxt = {}, [], res.best, res.nxt
    sys.setrecursionlimit(max(10000, sys.getrecursionlimit()))

    def frame(n):
        return prog.funcs[n].frame or 0

    def dfs(u):
        color[u] = GRAY
        stack.append(u)
        res.reachable.add(u)
        bv, bc = 0, None
        for v in sorted(prog.funcs[u].callees):
            if any(fnmatch.fnmatchcase(v, p) for p in ignore):
                continue
            c = color.get(v, WHITE)
            if c == GRAY:                      # 回边 => 递归环
                res.cycles.append(stack[stack.index(v):] + [v])
                continue
            if c == WHITE:
                dfs(v)
            if best[v] > bv or bc is None:
                bv, bc = best[v], v
        best[u] = frame(u) + bv
        nxt[u] = bc
        stack.pop()
        color[u] = BLACK

    dfs(name)
    res.worst = best[name]
    return res


# ----------------------------------------------------------------------------
# 报告输出
# ----------------------------------------------------------------------------

SRC_MARK = {"su": "", "htm": "", "config": "ᶜ", "disasm": "*", None: "?"}


def fmt_frame(f):
    if f.frame is None:
        return yellow("?")
    s = "%d%s" % (f.frame, SRC_MARK.get(f.frame_src, ""))
    return red(s) if f.alloc == "dynamic" else s


def disp_name(f):
    extra = dim(" (+%d 别名)" % (len(f.aliases) - 1)) if len(f.aliases) > 1 else ""
    return f.name + extra


def section(title):
    print()
    print(bold(cyan("=" * 78)))
    print(bold(cyan(" " + title)))
    print(bold(cyan("=" * 78)))


def print_top(prog, n):
    section("单函数自身栈消耗 Top %d" % n)
    fs = sorted((f for f in prog.funcs.values() if f.frame is not None), key=lambda f: (-f.frame, f.name))[:n]
    print("  %-4s %8s  %-8s %-7s %s" % ("#", "字节", "类型", "来源", "函数"))
    for i, f in enumerate(fs, 1):
        typ = f.alloc or "-"
        typ_s = red("%-8s" % typ) if typ == "dynamic" else ("%-8s" % typ)
        loc = ""
        if f.su_files:
            loc = dim("  " + sorted(f.su_files)[0])
        elif f.htm_obj:
            loc = dim("  " + f.htm_obj)
        print("  %-4d %8d  %s %-7s %s%s" % (i, f.frame, typ_s, f.frame_src, disp_name(f), loc))


def print_chain(prog, res):
    last, _, _, _ = tree_glyphs()
    path = res.path()
    cyc_nodes = {n for c in res.cycles for n in c}
    for i, n in enumerate(path):
        f = prog.funcs[n]
        prefix = "" if i == 0 else "    " * (i - 1) + last
        kind = prog.funcs[path[i - 1]].callees.get(n) if i else None
        tag = ""
        if kind == "tail":
            tag += dim(" [尾调用]")
        elif kind == "cfg":
            tag += cyan(" [函数指针/配置]")
        elif kind == "const":
            tag += dim(" [常量 BLX]")
        if f.alloc == "dynamic":
            tag += red(" [动态栈!]")
        if n in cyc_nodes:
            tag += red(" [递归]")
        if f.indirect and not f.cfg_resolved:
            tag += yellow(" [含未解析间接调用]")
        print("  %s%s  [%s / %d]%s" % (prefix, bold(n) if i == 0 else n, fmt_frame(f), res.best[n], tag))


def print_full_tree(prog, res, max_depth):
    last, mid, bar, sp = tree_glyphs()
    printed = set()

    def rec(n, prefix, depth):
        kids = sorted((c for c in prog.funcs[n].callees if c in res.best), key=lambda c: -res.best[c])
        for i, c in enumerate(kids):
            is_last = i == len(kids) - 1
            f = prog.funcs[c]
            more = c in printed and prog.funcs[c].callees
            print("  %s%s%s  [%s / %d]%s" % (prefix, last if is_last else mid, c, fmt_frame(f),
                                             res.best[c], dim(" (见上)") if more else ""))
            if depth + 1 < max_depth and not more:
                printed.add(c)
                rec(c, prefix + (sp if is_last else bar), depth + 1)

    f = prog.funcs[res.name]
    print("  %s  [%s / %d]" % (bold(res.name), fmt_frame(f), res.best[res.name]))
    printed.add(res.name)
    rec(res.name, "", 0)


ETYPE_CN = {"normal": "普通入口", "isr": "中断(MSP)", "thread": "RTOS 线程(PSP)"}


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="ARM Cortex-M 编译期静态栈深度分析 (GCC .su / Keil .htm / objdump / fromelf)",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__.split("典型用法：")[1])
    ap.add_argument("--build-dir", default=".", help="构建目录/工程根目录（扫描 .su、uvprojx、构建脚本）")
    ap.add_argument("--elf", help=".elf / .axf 产物（缺省自动探测）")
    ap.add_argument("--htm", help="Keil armlink 调用图 .htm（缺省自动探测）")
    ap.add_argument("--no-htm", action="store_true", help="不使用 Keil .htm 数据")
    ap.add_argument("--su-dir", action="append", help="额外的 .su 目录（如 LTO 的 ltrans 输出目录），可多次指定")
    ap.add_argument("--disasm", help="直接使用已生成的反汇编文本（objdump -d 或 fromelf -c 输出）")
    ap.add_argument("--objdump", help="arm-none-eabi-objdump 路径")
    ap.add_argument("--fromelf", help="Keil fromelf 路径")
    ap.add_argument("--entry", action="append", default=[], help="普通入口函数（如 Reset_Handler / main），可多次")
    ap.add_argument("--isr", action="append", default=[], help="中断入口（加硬件压栈开销，计入 MSP）")
    ap.add_argument("--thread", action="append", default=[], help="RTOS 线程入口（加硬件压栈 + RTOS 上下文，PSP）")
    ap.add_argument("--auto-isr", action="store_true", help="自动把所有 *_IRQHandler / *_Handler 作为中断入口")
    ap.add_argument("--fpu", dest="fpu", action="store_true", default=None, help="按启用 FPU 计算异常压栈 (104B)")
    ap.add_argument("--no-fpu", dest="fpu", action="store_false", help="按无 FPU 计算异常压栈 (32B)")
    ap.add_argument("--align-pad", action="store_true", help="异常压栈额外计入 4 字节 STKALIGN 对齐填充")
    ap.add_argument("--rtos-ctx", type=int, help="RTOS 软件保存的上下文字节数（缺省: 无FPU 32 / FPU 100）")
    ap.add_argument("--config", help="stack_config.json（缺省自动查找 build-dir 与工具目录）")
    ap.add_argument("--limit", type=int, help="任意入口累计栈深上限（字节），超过则 exit(1)")
    ap.add_argument("--msp-limit", type=int, help="MSP 估算上限（字节），超过则 exit(1)")
    ap.add_argument("--strict", action="store_true", help="存在递归/动态栈/未解析间接调用/未知帧时也判失败")
    ap.add_argument("--top", type=int, default=15, help="Top N 排行榜数量")
    ap.add_argument("--full-tree", type=int, metavar="DEPTH", help="额外打印完整调用树（限制深度）")
    ap.add_argument("--list-fp", action="store_true", help="列出地址被引用的函数（候选函数指针目标）")
    ap.add_argument("--all-indirect", action="store_true", help="列出全镜像的间接调用（而不只是入口可达的）")
    ap.add_argument("--json", help="输出 JSON 报告到文件（便于 CI 归档）")
    ap.add_argument("--detect", action="store_true", help="只做工程/工具链探测并退出")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--ascii", action="store_true", help="树形使用 ASCII 字符")
    ap.add_argument("--version", action="version", version=VERSION)
    args = ap.parse_args(argv)

    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    Style.color = sys.stdout.isatty() and not args.no_color and not os.environ.get("NO_COLOR")
    Style.ascii = args.ascii
    if Style.color and os.name == "nt":
        os.system("")

    info = detect_project(args.build_dir)
    section("工程 / 工具链探测")
    print("  工程目录 : %s" % info["root"])
    print("  工具链   : %s" % (info["toolchain"] or "未识别"))
    if info.get("cpu"):
        print("  CPU      : %s   FPU: %s" % (info["cpu"], "是" if info["fpu"] else "否"))
    for n in info["notes"]:
        print("  - %s" % n)
    if args.detect:
        print("  ELF      : %s" % (info["elf"] or find_elf(args.build_dir)))
        print("  调用图   : %s" % info["htm"])
        return 0

    if not args.elf and not args.disasm:
        args.elf = info["elf"] or find_elf(args.build_dir)
    if args.elf and not os.path.isfile(args.elf):
        die("找不到 ELF: %s" % args.elf)

    cfg_path = args.config
    if not cfg_path:
        for c in (os.path.join(args.build_dir, "stack_config.json"),
                  os.path.join(os.path.dirname(os.path.abspath(__file__)), "stack_config.json")):
            if os.path.isfile(c):
                cfg_path = c
                break
    cfg = load_config(cfg_path)

    prog, sources = build_program(args, info, cfg)
    fpu = args.fpu if args.fpu is not None else bool(cfg.get("fpu", info.get("fpu") or False))
    hw = (HW_FRAME_FPU if fpu else HW_FRAME_BASIC) + (ALIGN_PAD if args.align_pad or cfg.get("align_pad") else 0)
    rtos_ctx = args.rtos_ctx if args.rtos_ctx is not None else cfg.get("rtos_context_bytes", 100 if fpu else 32)

    print("  ELF      : %s" % (args.elf or "-"))
    print("  反汇编   : %s" % (sources["disasm"] or yellow("不可用（仅用调用图/.su）")))
    print("  Keil 调用图: %s" % (sources["htm"] or "-"))
    print("  .su 文件 : %d 个" % sources["su_files"])
    print("  配置     : %s" % (cfg_path or "-"))
    cnt = {}
    for f in prog.funcs.values():
        cnt[f.frame_src] = cnt.get(f.frame_src, 0) + 1
    print("  帧来源   : " + ", ".join("%s=%d" % (k or "未知", v) for k, v in sorted(cnt.items(), key=lambda x: str(x[0]))))
    print("  异常压栈 : %d B (%s%s)   RTOS 上下文: %d B" % (hw, "FPU" if fpu else "无 FPU",
                                                       " + 对齐" if hw % 8 else "", rtos_ctx))

    # ---- 入口 ----
    ents = cfg.get("entries", {})
    normal = args.entry + ents.get("normal", [])
    isrs = args.isr + ents.get("isr", [])
    threads = args.thread + ents.get("thread", [])
    if args.auto_isr or cfg.get("auto_isr"):
        for f in sorted(prog.funcs.values(), key=lambda x: x.name):
            if any(re.search(r"(_IRQHandler|_Handler)$", a) for a in f.aliases) and "Reset_Handler" not in f.aliases:
                isrs.append(f.name)
    if not (normal or isrs or threads):
        normal = [n for n in ("Reset_Handler", "main") if prog.resolve(n)][:1]
    ignore = cfg.get("ignore_functions", [])

    results, seen = [], set()
    for etype, names in (("normal", normal), ("isr", isrs), ("thread", threads)):
        for n in names:
            f = prog.resolve(n)
            if f is None:
                sug = difflib.get_close_matches(n, list(prog.alias), n=3)
                die("入口函数不存在: %s%s" % (n, ("（是否指: %s）" % ", ".join(sug)) if sug else ""))
            if (etype, f.name) in seen:
                continue
            seen.add((etype, f.name))
            ov = {"normal": 0, "isr": hw, "thread": hw + rtos_ctx}[etype]
            r = analyze_entry(prog, f.name, etype, ov, ignore)
            cfg_lim = cfg.get("limits", {}).get(n, cfg.get("limits", {}).get(f.name))
            if cfg_lim is not None and args.limit is not None:
                r.limit = min(cfg_lim, args.limit)
            else:
                r.limit = cfg_lim if cfg_lim is not None else args.limit
            results.append(r)

    print_top(prog, args.top)

    # ---- 最坏调用链 ----
    section("最坏调用链 (Critical Path)   格式: 函数 [自身栈 / 累计栈]")
    print(dim("  标记: *=反汇编推算  ᶜ=配置覆盖  ?=未知帧(按0计)  红色=动态栈"))
    for r in results:
        if r.etype == "isr" and r.worst == 0 and not prog.funcs[r.name].callees:
            continue
        print()
        head = "  ▶ %s  <%s>  最坏栈深 %d B" % (r.name, ETYPE_CN[r.etype], r.worst)
        if r.overhead:
            head += "  + 开销 %d B = %s" % (r.overhead, bold("%d B" % r.total))
        if r.cycles:
            head += "  " + red("UNBOUNDED (递归调用)")
        print(bold(head))
        print_chain(prog, r)
        if args.full_tree:
            print(dim("  -- 完整调用树 (深度 %d) --" % args.full_tree))
            print_full_tree(prog, r, args.full_tree)

    # ---- 汇总 ----
    section("入口汇总")
    print("  %-34s %-15s %8s %6s %8s %8s  %s" % ("入口", "类型", "栈深", "开销", "合计", "上限", "状态"))
    failed = False
    reach_all = set()
    for r in results:
        reach_all |= r.reachable
        unknown = [n for n in r.reachable if prog.funcs[n].frame is None]
        dyn = [n for n in r.reachable if prog.funcs[n].alloc == "dynamic"]
        unres = [n for n in r.reachable if prog.funcs[n].indirect and not prog.funcs[n].cfg_resolved]
        flags = []
        if r.cycles:
            flags.append(red("递归"))
        if dyn:
            flags.append(red("动态栈"))
        if unres:
            flags.append(yellow("间接调用%d" % len(unres)))
        if unknown:
            flags.append(yellow("未知帧%d" % len(unknown)))
        status = green("OK")
        if r.limit is not None and r.total > r.limit:
            status, failed = red("超限!"), True
        elif args.strict and (r.cycles or dyn or unres or unknown):
            status, failed = red("STRICT 失败"), True
        elif flags:
            status = yellow("OK(下界)")
        print("  %-34s %-15s %8d %6d %8d %8s  %s %s" % (
            r.name[:34], ETYPE_CN[r.etype], r.worst, r.overhead, r.total,
            r.limit if r.limit is not None else "-", status, " ".join(flags)))

    # ---- MSP 估算 ----
    isr_res = [r for r in results if r.etype == "isr"]
    msp_total = None
    if isr_res:
        prio = cfg.get("isr_priority", {})
        groups = {}
        for r in isr_res:
            p = prio.get(r.name)
            key = ("p", p) if p is not None else ("solo", r.name)
            groups[key] = max(groups.get(key, 0), r.total)
        base = max([r.total for r in results if r.etype == "normal"] or [0])
        nest = sum(groups.values())
        msp_total = base + nest
        print()
        print("  MSP 估算 = 普通入口最大 %d + 中断嵌套 %d = %s" % (base, nest, bold("%d B" % msp_total)))
        if not prio:
            print(dim("    (未提供 isr_priority，按所有中断互相嵌套的最坏情况累加；"
                      "在配置中提供优先级可得到更紧的估算)"))
        if info.get("startup_stack"):
            sz, where = info["startup_stack"]
            ok = msp_total <= sz
            print("    启动文件定义主栈: %d B (%s) -> %s" % (sz, where, green("足够") if ok else red("可能不足")))
        msp_limit = args.msp_limit or cfg.get("msp_limit")
        if msp_limit and msp_total > msp_limit:
            print(red("    MSP 超过上限 %d B" % msp_limit))
            failed = True
    if sources.get("htm_max") is not None:
        print(dim("  (armlink 调用图报告的最大栈深: %d B，可交叉验证)" % sources["htm_max"]))

    # ---- 警告 ----
    section("警告")
    any_warn = False
    for r in results:
        for c in r.cycles:
            any_warn = True
            print("  " + red("[递归] ") + "%s: %s" % (r.name, " -> ".join(c)))
    for f in sorted(prog.funcs.values(), key=lambda x: x.name):
        if f.alloc == "dynamic" and (f.name in reach_all or not results):
            any_warn = True
            print("  " + red("[动态栈] ") + "%s 存在无界运行时栈分配 (VLA/alloca)，帧大小 %s 仅为下界"
                  % (f.name, f.frame))
    scope = prog.funcs.values() if args.all_indirect else [prog.funcs[n] for n in reach_all]
    ind = [f for f in sorted(scope, key=lambda x: x.name) if f.indirect]
    for f in ind:
        if f.cfg_resolved:
            print("  " + dim("[间接调用-已配置] %s -> %s" % (
                f.name, ", ".join(c for c, k in f.callees.items() if k == "cfg"))))
            continue
        any_warn = True
        for addr, txt in f.indirect:
            print("  " + yellow("[间接调用] ") + "%s @0x%08x: %s" % (f.name, addr, txt.split(";")[0].strip()))
    unknown = sorted(n for n in reach_all if prog.funcs[n].frame is None)
    if unknown:
        any_warn = True
        print("  " + yellow("[未知帧] ") + "以下函数无帧信息（按 0 计，可在 frame_override 中补充）: " + ", ".join(unknown))
    if ind and any(not f.cfg_resolved for f in ind):
        print(dim("  提示: 在 stack_config.json 的 \"indirect_calls\" 中补充 {\"调用者\": [\"目标\"]}；"
                  "不关心的库内部可加入 \"ignore_functions\""))
    if not any_warn:
        print("  " + green("无"))

    if args.list_fp:
        section("地址被引用的函数（候选函数指针目标）")
        for n in sorted(prog.addr_taken):
            print("  %s  [%s]" % (n, fmt_frame(prog.funcs[n])))

    if args.json:
        rep = {
            "version": VERSION, "elf": args.elf, "sources": sources, "fpu": fpu, "hw_frame": hw,
            "rtos_ctx": rtos_ctx, "msp_total": msp_total,
            "entries": [{
                "name": r.name, "type": r.etype, "worst": r.worst, "overhead": r.overhead, "total": r.total,
                "limit": r.limit, "unbounded": bool(r.cycles), "cycles": r.cycles,
                "critical_path": [{"func": n, "self": prog.funcs[n].frame, "cum": r.best[n]} for n in r.path()],
            } for r in results],
            "functions": {f.name: {"frame": f.frame, "src": f.frame_src, "alloc": f.alloc,
                                   "callees": f.callees, "indirect": ["0x%08x %s" % x for x in f.indirect]}
                          for f in prog.funcs.values()},
        }
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, ensure_ascii=False, indent=2)
        print("\n  JSON 报告已写入 %s" % args.json)

    print()
    if failed:
        print(red("结果: FAIL（超过栈门限）"))
        return 1
    print(green("结果: PASS"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
