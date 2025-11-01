#!/bin/bash
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Logging funkton
log() {
        # definiere variablen
        local log_to_console="$1"
        local log_to_file="$2"
        local log_level="$3"
        local filename="delete_runs.log"
        local color="${NC}"
        #log log_level die wir definiert haben
        [ "$log_level" = "INFO" ] && color="${BLUE}"
        [ "$log_level" = "WARN" ] && color="${YELLOW}"
        [ "$log_level" = "ERROR" ] && color="${RED}"
        [ "$log_level" = "SUCCESS" ] && color="${GREEN}"  # Neu: Für Erfolgsmeldungen
        # Wir entfernen die ersten "zahl" Argumente
        # damit sind local log_to_console etc. mit gemeint.
        # Der Rest ($*) ist die Nachricht
        shift 3
        # Ausgabe auf Console, nur wenn $log_to_console "true" ist
        if [ "$log_to_console" = "true" ]; then
                echo -e "${color}[$log_level]${NC} $*" >&2
        fi

        # In die Log-Datei schreiben wenn true
        if [ "$log_to_file" = "true" ]; then
        #  > Überschreibe die Datei 
        # >> Append an die Datei anhängen
                echo "$(date): $log_level: $*" >> "${filename}"
                if [ $? -ne 0 ]; then
                        echo -e "${RED}[FEHLER]${NC} Konnte nicht in '${filename}' schreiben. Bitte Berechtigungen prüfen." >&2
                fi
        fi
        # How to use  log "true" "true" "INFO" "Github workflow Helper gestartet"
}

banner_counter() {
    local succeeded="$1"
    local failed="$2"
    local cancelled="$3"
    local active="$4"
    
    echo ""
    ausgabe "$BLUE" "╔════════════════════════════════════════╗"
    ausgabe "$BLUE" "║  Workflow Statistiken                  ║"
    ausgabe "$BLUE" "╠════════════════════════════════════════╣"
    printf "${BLUE}║${NC} ${GREEN}%-38s${NC} ${BLUE}║${NC}\n" "Erfolgreich:     $succeeded" >&2
    printf "${BLUE}║${NC} ${RED}%-38s${NC} ${BLUE}║${NC}\n" "Fehlgeschlagen:  $failed" >&2
    printf "${BLUE}║${NC} ${YELLOW}%-38s${NC} ${BLUE}║${NC}\n" "Abgebrochen:     $cancelled" >&2
    printf "${BLUE}║${NC} ${BLUE}%-38s${NC} ${BLUE}║${NC}\n" "Aktiv/Wartend:   $active" >&2
    ausgabe "$BLUE" "╚════════════════════════════════════════╝"
    echo ""
}

ausgabe() {
        # banner() {
        #     ausgabe "╔════════════════════════════════════════╗" "$BLUE"
        #     ausgabe "║  Github workflow Helper                ║" "$BLUE"
        #     ausgabe "╚════════════════════════════════════════╝" "$BLUE"
        #     echo ""
        # }
        # Ausgaben funktion um farbige Texte zu erzeugen
        local input_text="$2"
        local start_color="$1"
        echo -e "${start_color}${input_text}${NC}" >&2
}

get_active_workflows_json() {
        log "false" "true" "INFO" "Rufe 'gh workflow list' API auf"
        local json_data
        json_data=$(gh workflow list --limit 20 --json name,state,id --jq '.[] | select(.state == "active")')
        # Fehlerprüfung
        if [ $? -ne 0 ]; then
                log "true" "true" "ERROR" "Fehler beim Abrufen der Workflows. Ist 'gh' installiert und eingeloggt?"
                return 1
        fi
        if [ -z "$json_data" ]; then
                log "true" "true" "WARN" "Keine aktiven Workflows gefunden."
                return 1
        fi
        echo "$json_data"
        return 0
}

get_workflow_id_from_json() {
        local json_data="$1"
        local workflow_name="$2"

        # Sucht im JSON nach dem Namen und gibt nur die ID zurück
        echo "$json_data" | jq -r "select(.name == \"$workflow_name\") | .id"
}

prompt_user_for_workflow() {
        # Diese Funktion baut das Menü, verarbeitet die Auswahl
        # und zeigt für jeden ausgewählten Workflow die Statistiken und Aktionen
        local json_data="$1"
        local workflow_names=() # Namen aus dem JSON in ein Array laden
        local choice=""       # (Variable für 'select' hinzugefügt)
        # Laden der Namen aus dem JSON laden
        mapfile -t workflow_names < <(echo "$json_data" | jq -r '.name')
        # Option "Beenden" hinzufügen
        workflow_names+=("Beenden")
        ausgabe "$BLUE" "\nBitte einen Workflow auswählen:"
        PS3="Deine Wahl (Nummer): " # Prompt für das 'select' Menü
        # Diese Schleife läuft, bis du "Beenden" wählst
        while true; do
                COLUMNS=1
                select choice in "${workflow_names[@]}"; do
                # Abbruch bedingung "Beenden"
                if [ "$choice" = "Beenden" ]; then
                        log "true" "true" "INFO" "Auswahl beendet."
                        break 2 # Beendet beide 'select' und 'while' Schleifen
                fi
                if [ -n "$choice" ]; then
                        local selected_id=$(get_workflow_id_from_json "$json_data" "$choice")
                        log "true" "false" "SUCCESS" "Workflow '$choice' (ID: $selected_id) ausgewählt."
                        
                        # Zeige Statistiken
                        get_workflow_run_stats "$selected_id"
                        
                        # Zeige Aktionen
                        prompt_workflow_actions "$selected_id"
                        
                        # Zurück zum Menü
                        ausgabe "$BLUE" "\nBitte einen Workflow auswählen:"
                        break 
                # ungültige Zahl
                else
                        ausgabe "$RED" "Ungültige Auswahl. Bitte erneut versuchen."
                        break
                fi
                done
        done
}

show_workflow_runs() {
    # Definiere Variablen (wie in deinem Stil)
    local workflow_id="$1"
    local limit="${2:-10}"  # Optional: Anzahl der Runs (default 10)
    local json_runs
    log "true" "true" "INFO" "Zeige Runs für Workflow-ID: $workflow_id (Limit: $limit)"  # Logge den Start
    # Rufe die Runs ab (mit --workflow filtert nach ID)
    json_runs=$(gh run list --workflow "$workflow_id" --limit "$limit" --json databaseId,status,createdAt,headSha)
    
    # Fehlerprüfung
    if [ $? -ne 0 ]; then
        log "true" "true" "ERROR" "Fehler beim Abrufen der Runs für Workflow-ID $workflow_id. Ist 'gh' installiert und eingeloggt?"
        return 1
    fi
    
    # Prüfe, ob Runs vorhanden sind (Array-Länge)
    if [ "$(echo "$json_runs" | jq '. | length')" -eq 0 ]; then
        log "true" "true" "WARN" "Keine Runs gefunden für Workflow-ID $workflow_id."
        return 1
    fi
    
    # Runs anzeigen (formatiert für Lesbarkeit)
    echo "Runs für Workflow-ID $workflow_id:"
    echo "$json_runs" | jq -r '.[] | "ID: \(.databaseId), Status: \(.status), Erstellt: \(.createdAt), SHA: \(.headSha)"'
    
    # Optional: Logge Erfolg
    log "true" "true" "INFO" "Runs erfolgreich angezeigt für Workflow-ID $workflow_id."
    return 0
}

prompt_user_for_runs() {
    local workflow_id="$1"
    local limit="${2:-10}"
    local json_runs
    local run_options=()
    local selected_run_ids=()
    local choice=""

    log "true" "true" "INFO" "Lade Runs für Workflow-ID: $workflow_id"
    # WICHTIG: Wir brauchen 'conclusion' für success/failure
    json_runs=$(gh run list --workflow "$workflow_id" --limit "$limit" --json databaseId,status,conclusion,createdAt,headSha)

    if [ $? -ne 0 ]; then
        log "true" "true" "ERROR" "Fehler beim Abrufen der Runs für Workflow-ID $workflow_id."
        return 1
    fi

    if [ "$(echo "$json_runs" | jq '. | length')" -eq 0 ]; then
        log "true" "true" "WARN" "Keine Runs gefunden für Workflow-ID $workflow_id."
        return 1
    fi

    # Runs in Array laden für Menü
    mapfile -t run_options < <(echo "$json_runs" | jq -r '"ID: \(.databaseId) - Status: \(.status) (\(.conclusion)) - SHA: \(.headSha)"')
    run_options+=("Beenden")

    ausgabe "$BLUE" "\nBitte Runs zum Löschen auswählen (mehrere möglich):"
    PS3="Deine Wahl (Nummer): "

    while true; do
        COLUMNS=1  # Setze COLUMNS direkt vor select
        select choice in "${run_options[@]}"; do
            if [ "$choice" = "Beenden" ]; then
                log "true" "true" "INFO" "Auswahl der Runs beendet."
                break 2
            fi

            if [ -n "$choice" ]; then
                # Extrahiere ID aus der Auswahl (z.B. "ID: 123 - ..." -> 123)
                run_id=$(echo "$choice" | sed 's/ID: \([0-9]*\).*/\1/')
                selected_run_ids+=("$run_id")
                log "true" "false" "SUCCESS" "Run '$choice' (ID: $run_id) zum Löschen hinzugefügt."
                break
            else
                ausgabe "$RED" "Ungültige Auswahl. Bitte erneut versuchen."
                break
            fi
        done
    done

    # Ausgewählte IDs ausgeben
    if [ ${#selected_run_ids[@]} -gt 0 ]; then
        for id in "${selected_run_ids[@]}"; do
            echo "$id"
        done
        return 0
    else
        return 1
    fi
}

# ==================================================================
# NEUE FUNKTIONEN START
# ==================================================================

get_workflow_run_stats() {
    local workflow_id="$1"
    log "true" "true" "INFO" "Zähle Runs für Workflow-ID: $workflow_id (Limit 1000)"
    
    local json_data
    # Wir brauchen 'status' (aktiv) und 'conclusion' (beendet)
    json_data=$(gh run list --workflow "$workflow_id" --limit 1000 --json status,conclusion)
    
    if [ $? -ne 0 ]; then
        log "true" "true" "ERROR" "Konnte Run-Status nicht abrufen."
        return 1
    fi
    # Zähle die verschiedenen Status mit jq
    local succeeded=$(echo "$json_data" | jq '[.[] | select(.conclusion == "success")] | length')
    local failed=$(echo "$json_data" | jq '[.[] | select(.conclusion == "failure")] | length')
    local cancelled=$(echo "$json_data" | jq '[.[] | select(.conclusion == "cancelled")] | length')
    local active=$(echo "$json_data" | jq '[.[] | select(.status == "queued" or .status == "in_progress")] | length')

    # Zeige Statistiken im Banner-Format
    banner_counter "$succeeded" "$failed" "$cancelled" "$active"
}

delete_runs_by_status() {
    local workflow_id="$1"
    local conclusion_status="$2" # "failure", "cancelled", "success"
    log "true" "true" "INFO" "Suche nach Runs mit Status '$conclusion_status'..."
    local run_ids
    run_ids=$(gh run list --workflow "$workflow_id" --limit 1000 --json databaseId,conclusion --jq ".[] | select(.conclusion == \"$conclusion_status\") | .databaseId")
    if [ -z "$run_ids" ]; then
        log "true" "true" "INFO" "Keine Runs mit Status '$conclusion_status' gefunden."
        return
    fi    
    local count=$(echo "$run_ids" | wc -l)
    log "true" "true" "WARN" "$count Runs mit Status '$conclusion_status' gefunden. Lösche sie jetzt..."
    
    # Lösche alle gefundenen Runs
    echo "$run_ids" | while read -r run_id; do
        if [ -n "$run_id" ]; then
            gh run delete "$run_id"
        fi
    done
    log "true" "true" "SUCCESS" "$count Runs gelöscht."
}

prompt_workflow_actions() {
    local workflow_id="$1"
    local choice=""
    
    # Endlosschleife, bis "Zurück" gewählt wird
    while true; do
        ausgabe "$BLUE" "\nAktionen für Workflow $workflow_id:"
        local options=(
            "Alle 'failed' Runs löschen"
            "Alle 'cancelled' Runs löschen"
            "Alle 'success' Runs löschen"
            "Interaktiv einzelne Runs zum Löschen auswählen (Limit 10)"
            "Letzte 10 Runs anzeigen"
            "Statistiken neu laden"
            "Zurück (nächster Workflow / Hauptmenü)"
        )
        PS3="Deine Wahl: "
        
        COLUMNS=1  # Setze COLUMNS direkt vor select
        select choice in "${options[@]}"; do
            case "$choice" in
                "Alle 'failed' Runs löschen")
                    delete_runs_by_status "$workflow_id" "failure"
                    break # Zurück zum "Aktionen"-Menü
                    ;;
                    
                "Alle 'cancelled' Runs löschen")
                    delete_runs_by_status "$workflow_id" "cancelled"
                    break # Zurück zum "Aktionen"-Menü
                    ;;

                "Alle 'success' Runs löschen")
                    delete_runs_by_status "$workflow_id" "success"
                    break # Zurück zum "Aktionen"-Menü
                    ;;
                    
                "Interaktiv einzelne Runs zum Löschen auswählen (Limit 10)")
                    # Rufe deine existierende Funktion auf
                    local selected_run_ids=()
                    mapfile -t selected_run_ids < <(prompt_user_for_runs "$workflow_id")
                    
                    if [ ${#selected_run_ids[@]} -gt 0 ]; then
                        log "true" "true" "INFO" "Lösche ${#selected_run_ids[@]} ausgewählte Runs..."
                        for run_id in "${selected_run_ids[@]}"; do
                            # Filtert wieder Log-Zeilen raus, falls mapfile sie aufnimmt
                            if ! [[ "$run_id" =~ ^[0-9]+$ ]]; then continue; fi
                            
                            log "true" "true" "INFO" "Lösche Run ID: $run_id"
                            gh run delete "$run_id"
                        done
                        log "true" "true" "SUCCESS" "Ausgewählte Runs gelöscht."
                    else
                        log "true" "true" "INFO" "Keine Runs zum Löschen ausgewählt."
                    fi
                    break # Zurück zum "Aktionen"-Menü
                    ;;
                    
                "Letzte 10 Runs anzeigen")
                    show_workflow_runs "$workflow_id" 10
                    # Bleibe im "Aktionen"-Menü (kein 'break')
                    # Zeige Menü-Prompt erneut an
                    ausgabe "$BLUE" "\nAktionen für Workflow $workflow_id:"
                    ;;

                "Statistiken neu laden")
                    get_workflow_run_stats "$workflow_id"
                    break # Zurück zum "Aktionen"-Menü
                    ;;

                "Zurück (nächster Workflow / Hauptmenü)")
                    break 2 # Beendet die 'select' und 'while' Schleife
                    ;;
                    
                *)
                    ausgabe "$RED" "Ungültige Auswahl."
                    break # Zurück zum "Aktionen"-Menü (select)
                    ;;

            esac
        done
    done
}

# ==================================================================
# NEUE FUNKTIONEN ENDE
# ==================================================================

start_date="2025-10-30"
end_date="2025-10-31"
current_date="$start_date"

ausgabe "$BLUE" "╔════════════════════════════════════════╗"
ausgabe "$BLUE" "║  Github workflow Helper                ║"
ausgabe "$BLUE" "╚════════════════════════════════════════╝"
echo ""

log "true" "false" "INFO" "Lade aktive Workflows..."
workflow_json_cache=$(get_active_workflows_json)
exit_status=$? 

if [ $exit_status -ne 0 ]; then
    log "true" "false" "ERROR" "Skript wird aufgrund eines Fehlers beim Laden beendet."
    exit 1
fi

# Starte die Workflow-Auswahl und Verarbeitung
prompt_user_for_workflow "$workflow_json_cache"

log "true" "true" "INFO" "Skript beendet."
