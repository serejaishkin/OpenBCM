#!/usr/bin/env python3
"""clang wrapper: redirect every reference to Xcode's (new) SDK onto an older SDK.

usage (via build.sh): clang_sdk_wrapper.py REAL_CLANG OLD_SDK NEW_SDK[:NEW_SDK2...] -- clang args...
Rewrites plain arguments and @response files, drops -ivfsstatcache (stat cache of the new SDK).
"""
import os, sys

real, old, news = sys.argv[1], sys.argv[2], [n for n in sys.argv[3].split(":") if n]
args = sys.argv[5:] if len(sys.argv) > 4 and sys.argv[4] == "--" else sys.argv[4:]

import re
SDK_RE = re.compile(r'[^\s"\']*/Platforms/MacOSX\.platform/Developer/SDKs/MacOSX[0-9.]*\.sdk')

def fix(text):
    text = SDK_RE.sub(lambda m: old, text)   # any Xcode macOS SDK spelling (MacOSX.sdk, MacOSX26.sdk, MacOSX26.2.sdk)
    for n in news:
        text = text.replace(n, old)
    return text

out, skip = [], False
for a in args:
    if skip:
        skip = False
        continue
    if a == "-ivfsstatcache":
        skip = True
        continue
    if a.startswith("@") and os.path.isfile(a[1:]):
        src = a[1:]
        with open(src, errors="surrogateescape") as f:
            body = fix(f.read())
        # drop -ivfsstatcache <path> inside response files too
        lines = body.split("\n")
        keep, i = [], 0
        while i < len(lines):
            if lines[i].strip() == "-ivfsstatcache":
                i += 2
                continue
            keep.append(lines[i]); i += 1
        dst = src + ".oldsdk"
        with open(dst, "w", errors="surrogateescape") as f:
            f.write("\n".join(keep))
        out.append("@" + dst)
        continue
    out.append(fix(a))
final = [real] + out + ["-isysroot", old]
try:
    with open(os.environ.get("OPENBCM_WRAPPER_LOG", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build", "wrapper.log")), "a") as lf:
        left = [x for x in final if SDK_RE.search(x.replace(old, ''))]
        lf.write("CALL %d args, %d still mention new SDK, cwd=%s\n  in : %s\n  out: %s\n" % (
            len(final), len(left), os.getcwd(), " ".join(args)[:600], " ".join(final)[:600]))
except Exception:
    pass
import subprocess
logp = os.environ.get("OPENBCM_WRAPPER_LOG", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build", "wrapper.log"))
r = subprocess.run(final, stdin=subprocess.DEVNULL)
if r.returncode != 0:
    try:
        v = subprocess.run(final + ["-v"], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                           stderr=subprocess.PIPE, universal_newlines=True, errors="replace")
        keep = [l for l in v.stderr.splitlines()
                if l.startswith(" /") or "search starts" in l or "End of search" in l
                or l.lstrip().startswith('"') or "sysroot" in l]
        with open(logp, "a") as lf:
            lf.write("FAILED rc=%d\n  FULL ARGV: %s\n  -v:\n%s\n" % (r.returncode, " ".join(final), "\n".join(k[:3000] for k in keep)))
    except Exception as e:
        with open(logp, "a") as lf:
            lf.write("FAILED rc=%d (diag error %r)\n" % (r.returncode, e))
sys.exit(r.returncode)
