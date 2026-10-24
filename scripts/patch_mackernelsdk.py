#!/usr/bin/env python3
"""Make pinned MacKernelSDK libkern.h compile against macOS 26 SDK. Idempotent."""
import sys, os
p = os.path.join(sys.argv[1] if len(sys.argv) > 1 else "MacKernelSDK", "Headers/libkern/libkern.h")
s = open(p).read()
if "OPENBCM-PATCH" in s:
    sys.exit(0)
for d in ("extern int      ffsll(unsigned long long);", "extern int      fls(unsigned int);",
          "extern int      flsll(unsigned long long);"):
    s = s.replace(d, "/* OPENBCM-PATCH: declared by the SDK's <strings.h> */ // " + d)
shim = """#include <os/base.h>
#include <mach/vm_types.h>
#ifndef OS_WARN_RESULT
#define OS_WARN_RESULT __attribute__((__warn_unused_result__))
#endif
typedef uint64_t addr64_t; /* OPENBCM-PATCH */
"""
s = s.replace("#include <libkern/crc.h>\n", "#include <libkern/crc.h>\n" + shim, 1)
open(p, "w").write(s)
print("patched", p)
