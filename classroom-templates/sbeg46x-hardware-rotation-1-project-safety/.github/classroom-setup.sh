#!/bin/sh
# Installs this lab's packages inside the GitHub Classroom grader (do not edit).
set -e
need_cxx=no
# C++ track chosen in LANGUAGE, or a package without a prebuilt wheel (toppra) that pip must compile
if [ -f LANGUAGE ] && head -n 1 LANGUAGE | grep -qiE '^ *(cpp|c\+\+)'; then need_cxx=yes; fi
if grep -qiE '^toppra' requirements.txt; then need_cxx=yes; fi
if [ "$need_cxx" = yes ]; then
  echo "Installing the C++ compiler"
  apt-get update -qq && apt-get install -y -qq --no-install-recommends g++ > /dev/null
fi
grep -viE '^pytest([=<>~ ]|$)' requirements.txt > /tmp/requirements-grader.txt || true
python -m pip install --quiet --no-cache-dir --disable-pip-version-check -r /tmp/requirements-grader.txt
