#!/usr/bin/env python3
"""Announce a GitHub Release (or any update) on LinkedIn.

This is the core of the **gh-release-to-linkedin** tool. It works in two modes:

  1. As a GitHub Action step (triggered on ``release: published``), reading the
     release metadata from environment variables.
  2. As a local CLI, reading the same environment variables but allowing every
     value to be overridden with command-line flags.

It converts the release notes (Markdown) into clean LinkedIn-friendly plaintext
and publishes a post to the member's LinkedIn profile via the LinkedIn UGC Posts
API.

Nothing here is hardcoded to a specific project: hashtags, repository URL,
title, body and URL are all configurable inputs with sensible defaults.

Environment variables
---------------------
RELEASE_TITLE          : Release name / title.
RELEASE_BODY           : Release notes (Markdown).
RELEASE_URL            : URL of the release page.
REPO_URL               : Repository URL. Defaults to
                         ``$GITHUB_SERVER_URL/$GITHUB_REPOSITORY`` when running
                         inside GitHub Actions, otherwise empty.
HASHTAGS               : Space-separated hashtags appended to the post
                         (default: empty).
LINKEDIN_ACCESS_TOKEN  : OAuth 2.0 access token with ``w_member_social`` scope.
LINKEDIN_AUTHOR_URN    : Author URN, e.g. ``urn:li:person:xxxx``.
DRY_RUN                : If truthy, print the post + payload and do NOT call the
                         LinkedIn API.

CLI flags (override the env vars above)
---------------------------------------
--title, --body, --url, --repo-url, --hashtags, --dry-run

Dependencies: Python 3 stdlib + ``requests``.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys

import requests

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# LinkedIn UGC Posts endpoint (classic v2 API). It works with a personal
# access token that carries the ``w_member_social`` scope.
#
# NOTE: LinkedIn also offers a newer *versioned* endpoint
#   https://api.linkedin.com/rest/posts
# which requires an extra ``LinkedIn-Version: YYYYMM`` header (and a slightly
# different request body schema). We use the stable v2 ``ugcPosts`` endpoint
# here because it is the simplest path for a single-member
# "Share on LinkedIn" app.
LINKEDIN_UGC_POSTS_URL = "https://api.linkedin.com/v2/ugcPosts"

# LinkedIn hard caps share commentary at ~3000 characters. We keep a margin.
MAX_POST_CHARS = 3000


# ---------------------------------------------------------------------------
# Markdown -> plaintext helpers
# ---------------------------------------------------------------------------

def _markdown_to_plaintext(md: str) -> str:
    """Convert a small subset of Markdown into LinkedIn-friendly plaintext.

    Transformations:
      * Strip ATX headers ('#', '##', ...) but keep the heading text.
      * Convert '- ' / '* ' bullet markers into '• '.
      * Collapse links '[text](url)' -> 'text (url)' (or just 'text' when the
        text already equals the URL).
      * Remove bold/italic markers ('**', '__', '*', '_') keeping inner text.
      * Strip inline-code/code-fence backticks but keep the inner text.
      * Drop horizontal rules ('---', '***').
      * Emojis and ordinary punctuation are preserved.

    The goal is "clean enough" plaintext, not a full CommonMark renderer.
    """
    if not md:
        return ""

    text = md.replace("\r\n", "\n").replace("\r", "\n")

    # Remove fenced code blocks' fences (```), keep the inner lines as text.
    text = re.sub(r"^```.*$", "", text, flags=re.MULTILINE)

    out_lines = []
    for raw_line in text.split("\n"):
        line = raw_line.rstrip()

        # Horizontal rules -> drop entirely.
        if re.fullmatch(r"\s*([-*_])\1{2,}\s*", line):
            out_lines.append("")
            continue

        # ATX headers: strip leading '#' chars and any trailing '#'.
        line = re.sub(r"^\s{0,3}#{1,6}\s*", "", line)
        line = re.sub(r"\s*#+\s*$", "", line)

        # Bullet markers '- ' / '* ' / '+ ' -> '• ' (preserve indentation).
        m = re.match(r"^(\s*)[-*+]\s+(.*)$", line)
        if m:
            indent, content = m.group(1), m.group(2)
            line = f"{indent}• {content}"

        out_lines.append(line)

    text = "\n".join(out_lines)

    # Links: [text](url) -> "text (url)" or just "text" when redundant.
    def _link_sub(match: "re.Match[str]") -> str:
        label = match.group(1).strip()
        url = match.group(2).strip()
        if not label or label == url:
            return url
        return f"{label} ({url})"

    text = re.sub(r"\[([^\]]*)\]\(([^)]+)\)", _link_sub, text)

    # Remove inline code backticks but keep the inner text.
    text = re.sub(r"`([^`]*)`", r"\1", text)
    text = text.replace("`", "")

    # Bold/italic markers. Order matters: handle the double markers first.
    text = re.sub(r"\*\*([^*]+)\*\*", r"\1", text)
    text = re.sub(r"__([^_]+)__", r"\1", text)
    text = re.sub(r"\*([^*]+)\*", r"\1", text)
    text = re.sub(r"(?<!\w)_([^_]+)_(?!\w)", r"\1", text)

    # Collapse 3+ consecutive blank lines into a single blank line.
    text = re.sub(r"\n{3,}", "\n\n", text)

    return text.strip()


# ---------------------------------------------------------------------------
# Post formatting
# ---------------------------------------------------------------------------

def format_post(title: str, body: str, release_url: str, repo_url: str,
                hashtags: str = "") -> str:
    """Build the final LinkedIn post text from release metadata.

    Layout::

        🚀 <title>

        <clean release notes>

        🔗 Release: <release_url>
        ⭐ Repo: <repo_url>

        <hashtags>

    Any empty section is omitted. The body is truncated gracefully (with an
    ellipsis) so the whole post stays under the LinkedIn ~3000 char limit.
    """
    title = (title or "").strip()
    repo_url = (repo_url or "").strip()
    release_url = (release_url or "").strip()
    hashtags = (hashtags or "").strip()

    headline = title if title else "New release"

    clean_body = _markdown_to_plaintext(body)

    # Build the fixed footer first so we know how much room the body gets.
    footer_lines = []
    if release_url:
        footer_lines.append(f"\U0001f517 Release: {release_url}")
    if repo_url and repo_url != release_url:
        footer_lines.append(f"⭐ Repo: {repo_url}")
    footer = "\n".join(footer_lines)

    header = f"\U0001f680 {headline}"

    # Assemble fixed (non-body) parts to compute the budget for the body.
    fixed_parts = [header]
    if footer:
        fixed_parts.append(footer)
    if hashtags:
        fixed_parts.append(hashtags)
    fixed_text = "\n\n".join(fixed_parts)

    # Reserve room: fixed text + 2 separators ("\n\n") around the body.
    separators_len = len("\n\n") * 2
    budget = MAX_POST_CHARS - len(fixed_text) - separators_len

    if clean_body and budget > 0:
        if len(clean_body) > budget:
            ellipsis = " …"
            cut = max(0, budget - len(ellipsis))
            # Try to cut on a word/line boundary for a cleaner truncation.
            snippet = clean_body[:cut]
            boundary = max(snippet.rfind("\n"), snippet.rfind(" "))
            if boundary > cut * 0.6:  # only snap back if not losing too much
                snippet = snippet[:boundary]
            clean_body = snippet.rstrip() + ellipsis
        parts = [header, clean_body]
    else:
        # No body (or no room for it): header + footer + hashtags only.
        parts = [header]

    if footer:
        parts.append(footer)
    if hashtags:
        parts.append(hashtags)
    post = "\n\n".join(parts)

    # Final hard safety clamp (should rarely trigger given the budget logic).
    if len(post) > MAX_POST_CHARS:
        post = post[:MAX_POST_CHARS - 1].rstrip() + "…"

    return post


# ---------------------------------------------------------------------------
# LinkedIn API call
# ---------------------------------------------------------------------------

def _build_payload(text: str, author_urn: str) -> dict:
    """Return the LinkedIn UGC Posts request body for ``text``."""
    return {
        "author": author_urn,
        "lifecycleState": "PUBLISHED",
        "specificContent": {
            "com.linkedin.ugc.ShareContent": {
                "shareCommentary": {"text": text},
                "shareMediaCategory": "NONE",
            }
        },
        "visibility": {
            "com.linkedin.ugc.MemberNetworkVisibility": "PUBLIC"
        },
    }


def post_to_linkedin(text: str, token: str, author_urn: str) -> dict:
    """Publish ``text`` as a LinkedIn UGC post and return the parsed response.

    Raises
    ------
    RuntimeError
        If LinkedIn responds with a non-2xx status code. The response body is
        included in the error message to aid debugging.
    """
    headers = {
        "Authorization": f"Bearer {token}",
        "X-Restli-Protocol-Version": "2.0.0",
        "Content-Type": "application/json",
    }

    payload = _build_payload(text, author_urn)

    resp = requests.post(
        LINKEDIN_UGC_POSTS_URL,
        headers=headers,
        data=json.dumps(payload),
        timeout=30,
    )

    if not (200 <= resp.status_code < 300):
        raise RuntimeError(
            f"LinkedIn API returned HTTP {resp.status_code}: {resp.text}"
        )

    # LinkedIn returns the created post id in the body and/or the
    # ``x-restli-id`` / ``x-linkedin-id`` response headers.
    try:
        data = resp.json()
    except ValueError:
        data = {}

    post_id = (
        data.get("id")
        or resp.headers.get("x-restli-id")
        or resp.headers.get("X-RestLi-Id")
        or resp.headers.get("x-linkedin-id")
    )
    if post_id and "id" not in data:
        data["id"] = post_id

    return data


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _is_truthy(value: "str | None") -> bool:
    """Return True for '1'/'true'/'yes'/'on' (case-insensitive)."""
    if value is None:
        return False
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _default_repo_url() -> str:
    """Derive the repo URL from GitHub Actions env vars, if present."""
    server = os.environ.get("GITHUB_SERVER_URL", "").strip()
    repo = os.environ.get("GITHUB_REPOSITORY", "").strip()
    if server and repo:
        return f"{server.rstrip('/')}/{repo}"
    return ""


def _parse_args(argv: "list[str] | None" = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a GitHub Release announcement to LinkedIn. "
                    "Values fall back to the matching environment variables.",
    )
    parser.add_argument("--title", help="Release title (env: RELEASE_TITLE).")
    parser.add_argument("--body", help="Release notes, Markdown (env: RELEASE_BODY).")
    parser.add_argument("--url", help="Release page URL (env: RELEASE_URL).")
    parser.add_argument("--repo-url", help="Repository URL (env: REPO_URL).")
    parser.add_argument("--hashtags", help="Space-separated hashtags (env: HASHTAGS).")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Preview the post + payload without calling LinkedIn "
             "(env: DRY_RUN).",
    )
    return parser.parse_args(argv)


def main(argv: "list[str] | None" = None) -> int:
    """Build the post and either print it (dry run) or publish it."""
    args = _parse_args(argv)

    # Env first; CLI flags override when provided.
    title = args.title if args.title is not None \
        else os.environ.get("RELEASE_TITLE", "")
    body = args.body if args.body is not None \
        else os.environ.get("RELEASE_BODY", "")
    release_url = args.url if args.url is not None \
        else os.environ.get("RELEASE_URL", "")
    repo_url = args.repo_url if args.repo_url is not None \
        else os.environ.get("REPO_URL", "") or _default_repo_url()
    hashtags = args.hashtags if args.hashtags is not None \
        else os.environ.get("HASHTAGS", "")

    token = os.environ.get("LINKEDIN_ACCESS_TOKEN", "")
    author_urn = os.environ.get("LINKEDIN_AUTHOR_URN", "")
    dry_run = args.dry_run or _is_truthy(os.environ.get("DRY_RUN"))

    post_text = format_post(title, body, release_url, repo_url, hashtags)

    if dry_run:
        print("=== DRY RUN: LinkedIn post would be ===\n")
        print(post_text)
        print("\n=== Request payload (preview) ===\n")
        preview_payload = _build_payload(
            post_text, author_urn or "<LINKEDIN_AUTHOR_URN not set>")
        print(json.dumps(preview_payload, ensure_ascii=False, indent=2))
        print(f"\n(Post length: {len(post_text)} chars)")
        return 0

    # Real run: validate credentials.
    missing = []
    if not token:
        missing.append("LINKEDIN_ACCESS_TOKEN")
    if not author_urn:
        missing.append("LINKEDIN_AUTHOR_URN")
    if missing:
        print(
            "ERROR: missing required environment variable(s): "
            + ", ".join(missing)
            + ".\nSet them as GitHub repo secrets, or run with --dry-run / "
            "DRY_RUN=1 to preview without posting.",
            file=sys.stderr,
        )
        return 1

    try:
        result = post_to_linkedin(post_text, token, author_urn)
    except Exception as exc:  # noqa: BLE001 - surface any failure clearly
        print(f"ERROR: failed to publish post to LinkedIn: {exc}",
              file=sys.stderr)
        return 1

    post_id = result.get("id", "<unknown>")
    print("Successfully published LinkedIn post.")
    print(f"Post id/URN: {post_id}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
