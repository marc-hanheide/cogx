#! /usr/bin/env python2.5
# -*- coding: utf-8 -*-

import os
import os.path
import re


RE_CITATION = re.compile(
    r"(?P<head>(\\cite|\\shortcite|\\citeyear|\\citeauthor)"
    r"(\[.*?\]|<.*?>)*"
    r"\{)(?P<keys>.*?)(?P<tail>\})", re.DOTALL)
RE_BIBLIOGRAPHY = re.compile(r"\\bibliography{(?P<bibfiles>.*?)}", re.DOTALL)
RE_LIST_ENTRY = re.compile(r"[^,\s]+", re.DOTALL)

class Citation(object):
    def __init__(self, citecmd, citecmd_span, key, key_span):
        self.citecmd = citecmd
        self.citecmd_span = citecmd_span
        self.key = key
        self.key_span = key_span

    def replace_key(self, text, new_key):
        def _replace(text, span):
            assert text[span[0]:span[1]] == self.key, text[span[0]:span[1]]
            return text[:span[0]] + new_key + text[span[1]:]
        text_from = self.citecmd_span[0] + self.key_span[0]
        text_to = self.citecmd_span[0] + self.key_span[1]
        new_text = _replace(text, (text_from, text_to))
        new_citecmd = _replace(self.citecmd, self.key_span)
        return new_text, new_citecmd

def comma_split(text):
    return [part.strip() for part in text.split(",")]

def uses_literatur_bib(filename):
    text = open(filename).read()
    for match in RE_BIBLIOGRAPHY.finditer(text):
        if "literatur" in comma_split(match.group("bibfiles")):
            return True
    return False

def relevant_tex_files(base_path="."):
    # A TeX file is relevant iff any tex file in the same directory
    # uses the literatur.bib file.
    for path, subdirs, filenames in os.walk(base_path):
        subdirs.sort()
        filenames.sort()
        tex_files = [os.path.join(path, filename) for filename in filenames
                     if os.path.splitext(filename)[1] == ".tex"]
        if any(uses_literatur_bib(tex_file) for tex_file in tex_files):
            for tex_file in tex_files:
                yield tex_file

def citations(text):
    for match in RE_CITATION.finditer(text):
        citecmd = match.group(0)
        citecmd_span = match.span()
        keys_offset = match.span("keys")[0] - match.span()[0]
        keys_text = match.group("keys")
        for key_match in RE_LIST_ENTRY.finditer(keys_text):
            key = key_match.group(0)
            key_span = (keys_offset + key_match.span()[0],
                        keys_offset + key_match.span()[1])
            yield Citation(citecmd, citecmd_span, key, key_span)

def replace_bib_key(filename, from_key, to_key):
    text = open(filename).read()

    hits = []
    for citation in citations(text):
        if citation.key == to_key:
            raise SystemExit("new key already in use in %s" % filename)
        elif citation.key == from_key:
            hits.append(citation)

    # Test that we do not have overlapping hits. This could happen
    # if we did silly stuff like \cite{ff,ff}.
    for hit, next_hit in zip(hits, hits[1:]):
        if not (hit.citecmd_span[1] <= next_hit.citecmd_span[0]):
            raise SystemExit("overlapping hits: %s vs. %s" %
                             hit.citecmd, next_hit.citecmd)

    messages = []
    for hit in reversed(hits):
        text, new_citecmd = hit.replace_key(text, to_key)
        messages.append("%s: %s => %s" % (filename, hit.citecmd, new_citecmd))

    for message in reversed(messages):
        print message

    if messages:
        open(filename, "w").write(text)

def main(args):
    if len(args) < 3:
        raise SystemExit("usage: %s FROM_KEY TO_KEY [PATH ...]" % args[0])

    from_key, to_key, paths = args[1], args[2], args[3:]
    if not paths:
        paths = ["."]
    for path in paths:
        for filename in relevant_tex_files(path):
            replace_bib_key(filename, from_key, to_key)

if __name__ == "__main__":
    import sys
    main(sys.argv)
