"""Scores of the GitHub Classroom grader runs: starter < 100 (and no grader error), solution = 100."""
import base64, json, os, sys
def res(k):
    v = os.environ.get(k, "")
    if not v: return None
    return json.loads(base64.b64decode(v))
def score(r):
    t = r.get("tests", [])
    return sum(x.get("score", 0) or 0 for x in t), len(t), r.get("status"), r.get("message", "")
bad = []
kind = os.environ["KIND"]
for k, want in (("PY_ST", "lt"), ("PY_SOL", "full"), ("CPP_ST", "lt"), ("CPP_SOL", "full")):
    r = res(k)
    if r is None: continue
    s, n, st, msg = score(r)
    print(f"{k:8s} score {s:6.1f}/100  tests {n:3d}  status {st} {msg}")
    if kind != "lab" and k == "PY_ST": want = "full" if kind == "library" else "ran"
    ok = n > 0 and st != "error" and (s >= 99.9 if want == "full" else s < 99.9 if want == "lt" else True)
    if not ok: bad.append(k)
if res("PY_ST") is None: bad.append("PY_ST missing")
print("PROBLEMS: " + ", ".join(bad) if bad else "all grader runs as expected")
sys.exit(1 if bad else 0)
