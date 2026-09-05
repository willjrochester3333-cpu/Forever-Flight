#!/usr/bin/env python3
"""Inline viewer/model.json into viewer/template.html -> viewer/index.html.

The Artifact CSP blocks fetch, so the model data has to ship inside the page.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V = os.path.join(ROOT, "viewer")

tpl = open(os.path.join(V, "template.html")).read()
data = open(os.path.join(V, "model.json")).read()
token = "/*__MODEL__*/null"
if token not in tpl:
    sys.exit("template.html is missing the /*__MODEL__*/null placeholder")
out = tpl.replace(token, data)
path = os.path.join(V, "index.html")
open(path, "w").write(out)
print(f"wrote viewer/index.html  {os.path.getsize(path)/1024:.0f} kB")
