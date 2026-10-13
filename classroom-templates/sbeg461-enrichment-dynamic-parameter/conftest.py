"""Selects the Python or the C++ auto-checks from the first line of the file LANGUAGE (python | cpp)."""
import os
from pathlib import Path


def _language(config):
    v = os.environ.get("LAB_LANGUAGE", "")
    f = Path(str(config.rootpath)) / "LANGUAGE"
    if not v and f.exists():
        words = f.read_text(encoding="utf-8").split()
        v = words[0] if words else ""
    return "cpp" if v.strip().lower() in ("cpp", "c++") else "python"


def pytest_ignore_collect(collection_path, config):
    p = Path(str(collection_path))
    if p.is_dir():
        return True if p.name in ("cpp_support", "build") else None
    if not (p.name.startswith("test_") and p.suffix == ".py"):
        return None
    cpp_test = p.name == "test_cpp_track.py"
    return True if (_language(config) == "cpp") != cpp_test else None
