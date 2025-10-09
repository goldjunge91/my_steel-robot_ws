#!/usr/bin/env python3

"""Hilfsprogramm zum Konvertieren von .repos (vcstool) Einträgen in Git-Submodule.

Dieses Skript sucht rekursiv unter dem Verzeichnis 'src' nach Dateien mit der Endung
'.repos' und fügt die dort angegebenen git-Repositories als Submodule in das
aktuelle Arbeitsverzeichnis ein.

Funktionen:
- setup_logging: Logging konfigurieren (STDOUT und optional Datei).
- parse_repos_file: .repos-Datei parsen und für jeden git-Eintrag add_git_submodule aufrufen.
- add_git_submodule: Führt 'git submodule add' aus (oder simuliert bei --dry-run).
- is_submodule: Prüft, ob ein Pfad bereits als Submodul registriert ist.

Hinweis:
Dieses Skript verändert Git-Metadaten (Submodule). Vor dem Einsatz in produktiven
Repositories empfiehlt sich ein Commit oder ein Sicherungs-Branch.
"""

import glob
import os
import sys
import subprocess
import yaml
import logging
import argparse
from datetime import datetime
from typing import List, Optional

prefix = "src"


def setup_logging(verbose: bool = False, log_file: Optional[str] = None):
    """Konfiguriert das Logging für das Skript.

    Parameter:
    - verbose: Wenn True, wird DEBUG-Level aktiviert, sonst INFO.
    - log_file: Optionaler Pfad zu einer Logdatei; None bedeutet nur Konsole.

    Seiteneffekt:
    - Setzt globales logging-Konfigurationsverhalten.
    """
    level = logging.DEBUG if verbose else logging.INFO
    fmt = "%(asctime)s %(levelname)-7s %(message)s"
    handlers: List[logging.Handler] = [logging.StreamHandler()]
    # Only create a FileHandler when an explicit path (not None) is provided
    if log_file is not None:
        handlers.append(logging.FileHandler(log_file))
    logging.basicConfig(level=level, format=fmt, handlers=handlers)


def add_git_submodule(repo_name, repo_url, repo_version, dry_run=False):
    """Fügt ein Git-Repository als Submodul hinzu.

    Parameter:
    - repo_name: Zielpfad/Name des Submoduls (relativ zum Arbeitsverzeichnis).
    - repo_url: Remote-URL des Repositories (SSH oder HTTPS).
    - repo_version: Branch/Tag/Commit, welcher ausgecheckt werden soll.
    - dry_run: Wenn True, werden die Git-Kommandos nur geloggt, nicht ausgeführt.

    Rückgabe:
    - True bei Erfolg (oder im Dry-Run), False bei Fehlern beim Aufruf von git.

    Seiteneffekt:
    - Führt 'git submodule add -b <version> <url> <name>' aus, sofern dry_run False.
    """
    cmd = ["git", "submodule", "add", "-b", str(repo_version), repo_url, repo_name]
    logging.info("Planned: %s -> url=%s version=%s", repo_name, repo_url, repo_version)
    logging.debug("Prepared git command: %s", " ".join(cmd))
    if dry_run:
        logging.info("[dry-run] Would run: %s", " ".join(cmd))
        return True
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.stdout:
        logging.debug("git stdout: %s", proc.stdout.strip())
    if proc.stderr:
        logging.debug("git stderr: %s", proc.stderr.strip())
    if proc.returncode != 0:
        logging.error(
            "git command failed with returncode=%d for %s. stderr: %s",
            proc.returncode,
            repo_name,
            proc.stderr.strip(),
        )
        return False
    logging.info("Added %s as a submodule.", repo_name)
    return True


# def add_git_submodule(repo_name, repo_url, repo_version dry_run=False):
# subprocess.call(['git', 'submodule', 'add', '-b', repo_version, repo_url, repo_name])


def is_submodule(repo_name):
    """Prüft, ob ein gegebener Pfad bereits als Git-Submodul registriert ist.

    Parameter:
    - repo_name: Pfad (relativ) des zu prüfenden Verzeichnisses.

    Rückgabe:
    - True, wenn das Verzeichnis als Submodul registriert ist, sonst False.

    Hinweis:
    - Implementiert über 'git submodule status <path>' und fängt Ausnahmen ab.
    """
    try:
        subprocess.check_output(
            ["git", "submodule", "status", repo_name], stderr=subprocess.DEVNULL
        )
        logging.debug("Checked submodule status for %s: present", repo_name)
        return True
    except subprocess.CalledProcessError:
        logging.debug("Checked submodule status for %s: not present", repo_name)
        return False


def parse_repos_file(file_path, dry_run=False):
    """Parst eine .repos-Datei und fügt enthaltene git-Repositories als Submodule hinzu.

    Parameter:
    - file_path: Pfad zur .repos-Datei.
    - dry_run: Wenn True, werden Aktionen nur geloggt und keine git-Aufrufe gemacht.

    Verhalten:
    - Liest YAML, extrahiert 'repositories' und verarbeitet nur Einträge mit type: git.
    - Überspringt ungültige Einträge oder Einträge ohne URL.
    - Prüft, ob Zielpfad bereits existiert oder schon als Submodul registriert ist.
    - Ruft add_git_submodule auf, um fehlende Submodule hinzuzufügen.
    """
    logging.info("Processing .repos file: %s", file_path)
    with open(file_path, "r") as file:
        repos_data = yaml.safe_load(file)
        if not repos_data:
            logging.warning("Empty or invalid .repos file: %s", file_path)
            return
        repositories = repos_data.get("repositories", {})

        for repo_name, repo_info in repositories.items():
            if not isinstance(repo_info, dict):
                logging.warning("Skipping invalid entry for %s", repo_name)
                continue
            if repo_info.get("type") != "git":
                logging.debug("Skipping non-git repository: %s", repo_name)
                continue

            if "type" in repo_info and repo_info["type"] == "git":
                repo_url = repo_info.get("url")
                if not repo_url:
                    logging.warning("Repository %s has no URL; skipping", repo_name)
                    continue
                repo_version = repo_info.get("version") or "main"
                submodule_name = os.path.join(prefix, repo_name)

                logging.info(
                    "Repo entry: name=%s url=%s version=%s target=%s",
                    repo_name,
                    repo_url,
                    repo_version,
                    submodule_name,
                )

                # Warn if target exists but is not a submodule
                if os.path.exists(submodule_name) and not is_submodule(submodule_name):
                    logging.warning(
                        "Target path exists but is not a git submodule: %s - skipping",
                        submodule_name,
                    )
                    continue

                if not is_submodule(submodule_name):
                    success = add_git_submodule(
                        submodule_name, repo_url, repo_version, dry_run=dry_run
                    )
                    if success:
                        logging.info("Added %s as a submodule.", repo_name)
                    else:
                        logging.warning("Failed to add %s as a submodule.", repo_name)


if __name__ == "__main__":
    """
    Hauptprogramm / CLI-Einstiegspunkt

    Dieses Kommentarfeld beschreibt das Verhalten, wenn das Skript als Programm
    ausgeführt wird (if __name__ == "__main__").

    Kurzbeschreibung:
    - Parst die Kommandozeilenargumente:
    --dry-run   : Aktionen nur protokollieren, keine Git-Kommandos ausführen.
    --verbose   : Aktiviert Debug-Logging.
    --force     : Fügt '--force' zu 'git submodule add' hinzu (monkey-patch zur Laufzeit).
    --log-file  : Optionaler Pfad zu einer Logdatei.
    - Initialisiert das Logging (setup_logging) mit den übergebenen Optionen.
    - Sucht rekursiv nach '*.repos' Dateien im Verzeichnis, das durch die Variable
    'prefix' (standardmäßig "src") angegeben ist.
    - Für jede gefundene .repos-Datei wird parse_repos_file aufgerufen, wobei
    --dry-run weitergereicht wird.
    - Bei Verwendung von --force wird zur Laufzeit eine Variante von
    add_git_submodule gesetzt, die '--force' an den git-Befehl anhängt.
    Hinweis zur Nutzung:
    - Vor Ausführung empfiehlt sich ein aktueller Commit oder ein Sicherungs-Branch,
    da das Skript Git-Submodule hinzufügt und damit Git-Metadaten ändert.
    - Im Fehlerfall werden die git-Fehlerausgaben geloggt; im Dry-Run werden keine
    Änderungen am Repository vorgenommen.
    """
    parser = argparse.ArgumentParser(description="Add git submodules from .repos files")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print actions without executing git commands",
    )
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Pass --force to git submodule add when needed",
    )
    parser.add_argument(
        "--log-file", type=str, default=None, help="Optional log file path"
    )
    args = parser.parse_args()

    setup_logging(verbose=args.verbose, log_file=args.log_file)
    logging.info("Starting repos_to_submodules.py at %s", datetime.now().isoformat())

    # Find .repos files within the src directory (use configurable prefix)
    # repos_files = glob.glob(os.path.join(prefix, '**', '*.repos'), recursive=True)
    repos_files = glob.glob(os.path.join(prefix, "**", "*.repos"), recursive=True)

    if not repos_files:
        logging.info("No .repos files found under %s", prefix)
    # If --force is requested we will include the option when running git
    # (the add_git_submodule function will be updated below if needed)
    for repos_file in repos_files:
        # pass dry_run through; force is handled by modifying the git command here
        if args.force:
            # monkey-patch add_git_submodule to include --force in the command
            old_add = add_git_submodule

            def add_git_submodule_force(
                repo_name, repo_url, repo_version, dry_run=False
            ):
                cmd = [
                    "git",
                    "submodule",
                    "add",
                    "--force",
                    "-b",
                    str(repo_version),
                    repo_url,
                    repo_name,
                ]
                logging.info(
                    "Planned: %s -> url=%s version=%s (with --force)",
                    repo_name,
                    repo_url,
                    repo_version,
                )
                logging.debug("Prepared git command: %s", " ".join(cmd))
                if dry_run:
                    logging.info("[dry-run] Would run: %s", " ".join(cmd))
                    return True
                proc = subprocess.run(cmd, capture_output=True, text=True)
                if proc.stdout:
                    logging.debug("git stdout: %s", proc.stdout.strip())
                if proc.stderr:
                    logging.debug("git stderr: %s", proc.stderr.strip())
                if proc.returncode != 0:
                    logging.error(
                        "git command failed with returncode=%d for %s. stderr: %s",
                        proc.returncode,
                        repo_name,
                        proc.stderr.strip(),
                    )
                    return False
                logging.info("Added %s as a submodule.", repo_name)
                return True

            # replace the function used in this run
            add_git_submodule = add_git_submodule_force
        parse_repos_file(repos_file, dry_run=args.dry_run)

    logging.info("Finished processing at %s", datetime.now().isoformat())
