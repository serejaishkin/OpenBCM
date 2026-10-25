#!/usr/bin/env python3
"""Make pinned MacKernelSDK libkern.h compile against macOS 26 SDK. Idempotent."""
import sys, os
p = os.path.join(sys.argv[1] if len(sys.argv) > 1 else "MacKernelSDK", "Headers/libkern/libkern.h")
s = open(p).read()
orig = s
DECLS = ("extern int      ffs(unsigned int);", "extern int      ffsll(unsigned long long);",
         "extern int      fls(unsigned int);", "extern int      flsll(unsigned long long);")
s = "".join(("/* OPENBCM-PATCH: declared by SDK <strings.h> */ // " + l) if l.rstrip("\n") in DECLS else l
            for l in s.splitlines(True))
shim = """#include <os/base.h>
#include <mach/vm_types.h>
#ifndef OS_WARN_RESULT
#define OS_WARN_RESULT __attribute__((__warn_unused_result__))
#endif
typedef uint64_t addr64_t; /* OPENBCM-PATCH */
"""
if "mach/vm_types.h" not in s:
    s = s.replace("#include <libkern/crc.h>\n", "#include <libkern/crc.h>\n" + shim, 1)
if s != orig:
    open(p, "w").write(s)
    print("patched", p)

# ---- IOKit/IOLocks.h: __DARWIN14_ALIAS no longer defined by the SDK ----
base = sys.argv[1] if len(sys.argv) > 1 else "MacKernelSDK"
p = os.path.join(base, "Headers/IOKit/IOLocks.h")
s = open(p).read()
if "OPENBCM-PATCH" not in s:
    s = s.replace("#include <IOKit/IOTypes.h>\n",
                  "#include <IOKit/IOTypes.h>\n#ifndef __DARWIN14_ALIAS\n#define __DARWIN14_ALIAS(x) /* OPENBCM-PATCH */\n#endif\n", 1)
    open(p, "w").write(s); print("patched", p)

# ---- IOKit/IOLib.h: Debugger() now declared (differently) by MacTypes.h ----
p = os.path.join(base, "Headers/IOKit/IOLib.h")
s = open(p).read()
d = "void Debugger(const char * reason);"
if d in s and "OPENBCM-PATCH" not in s:
    s = s.replace(d, "/* OPENBCM-PATCH: declared by MacTypes.h */ // " + d)
    open(p, "w").write(s); print("patched", p)
