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
                echo -e "${color}[$log_level]${NC} $*"
        fi

        # In die Log-Datei schreiben wenn true
        if [ "$log_to_file" = "true" ]; then
        #  > Überschreibe die Datei 
        # >> Append an die Datei anhängen
                echo "$(date): $log_level: $*" >> "${filename}"
                if [ $? -ne 0 ]; then
                        echo -e "${RED}[FEHLER]${NC} Konnte nicht in '${filename}' schreiben. Bitte Berechtigungen prüfen."
                fi
        fi
        # How to use  log "true" "true" "INFO" "Github workflow Helper gestartet"

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
        echo -e "${start_color}${input_text}${NC}"
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
        # und gibt die finalen IDs zurück.
        # Braucht: $1 = JSON-Daten
        local json_data="$1"
        local workflow_names=() # Namen aus dem JSON in ein Array laden
        local all_selected_ids=()  # Diese Variable sammelt alle IDs, die du auswählst
        local selected_id=""  # ID für den gewählten Namen finden
        local choice=""       # (Variable für 'select' hinzugefügt)
        COLUMNS=1
        # 1. ZUERST die Namen aus dem JSON laden
        mapfile -t workflow_names < <(echo "$json_data" | jq -r '.name')
        
        # 2. DANACH die Option "Fertig/Abbrechen" hinzufügen
        workflow_names+=("Fertig/Abbrechen")

        ausgabe "$BLUE" "\nBitte einen oder mehrere Workflows auswählen:"
        PS3="Deine Wahl (Nummer): " # Prompt für das 'select' Menü
        
        # Diese Schleife läuft, bis du "Fertig/Abbrechen" wählst
        while true; do
                select choice in "${workflow_names[@]}"; do
                
                # Abbruch bedingung "Fertig/Abbrechen"
                # (KORRIGIERTE LOGIK: Muss den exakten String prüfen)
                if [ "$choice" = "Fertig/Abbrechen" ]; then
                        log "true" "true" "INFO" "Auswahl beendet."
                        break 2 # Beendet beide 'select' und 'while' Schleifen
                fi

                # Korrekte Auswahl 
                if [ -n "$choice" ]; then
                        selected_id=$(get_workflow_id_from_json "$json_data" "$choice")
                        all_selected_ids+=("$selected_id") # Füge die ID zur Sammel-Liste hinzu
                        log "true" "false" "SUCCESS" "Workflow '$choice' (ID: $selected_id) hinzugefügt."
                        
                        # Wir müssen 'break' aufrufen, um das 'select' Menü 
                        # neu zu laden (und die 'while' Schleife fortzusetzen)
                        break 
                
                # ungültige Zahl ein
                else
                        ausgabe "$RED" "Ungültige Auswahl. Bitte erneut versuchen."
                        break
                fi
                done
        done

        # Alle gesammelten IDs ausgeben (jede in einer neuen Zeile)
        if [ ${#all_selected_ids[@]} -gt 0 ]; then
                for id in "${all_selected_ids[@]}"; do
                echo "$id"
                done
                return 0 # Erfolgscode
        else
                # "Abbrechen" gewählt, ohne etwas auszuwählen
                return 1 # Fehlercode
        fi
}

show_workflow_runs() {
    # Definiere Variablen (wie in deinem Stil)
    local workflow_id="$1"
    local limit="${2:-10}"  # Optional: Anzahl der Runs (default 10)
    local json_runs
    
    # Logge den Start
    log "true" "true" "INFO" "Zeige Runs für Workflow-ID: $workflow_id (Limit: $limit)"
    
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
    COLUMNS=1

    log "true" "true" "INFO" "Lade Runs für Workflow-ID: $workflow_id"
    json_runs=$(gh run list --workflow "$workflow_id" --limit "$limit" --json databaseId,status,createdAt,headSha)

    if [ $? -ne 0 ]; then
        log "true" "true" "ERROR" "Fehler beim Abrufen der Runs für Workflow-ID $workflow_id."
        return 1
    fi

    if [ "$(echo "$json_runs" | jq '. | length')" -eq 0 ]; then
        log "true" "true" "WARN" "Keine Runs gefunden für Workflow-ID $workflow_id."
        return 1
    fi

    # Runs in Array laden für Menü
    mapfile -t run_options < <(echo "$json_runs" | jq -r '"ID: \(.databaseId) - Status: \(.status) - SHA: \(.headSha)"')
    run_options+=("Fertig/Abbrechen")

    ausgabe "$BLUE" "\nBitte Runs zum Löschen auswählen (mehrere möglich):"
    PS3="Deine Wahl (Nummer): "

    while true; do
        select choice in "${run_options[@]}"; do
            if [ "$choice" = "Fertig/Abbrechen" ]; then
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

start_date="2025-10-30"
end_date="2025-10-31"
current_date="$start_date"

ausgabe "$BLUE" "╔════════════════════════════════════════╗"
ausgabe "$BLUE" "║  Github workflow Helper                ║"
ausgabe "$BLUE" "╚════════════════════════════════════════╝"
echo ""

log "true" "false" "INFO" "Lade aktive Workflows..."
workflow_json_cache=$(get_active_workflows_json)
exit_status=$? # Speichert den Exit-Code

# Prüfen, ob das Holen der Daten fehlgeschlagen ist (und 'workflow_json_cache' leer ist)
if [ $exit_status -ne 0 ]; then
    log "true" "false" "ERROR" "Skript wird aufgrund eines Fehlers beim Laden beendet."
    exit 1
fi

# (Dein 'selected_workflow_id=...' würde nur die erste ID speichern)
selected_id_list=()
mapfile -t selected_id_list < <(prompt_user_for_workflow "$workflow_json_cache")
exit_status=$? 

# Prüfen, ob der Benutzer "Abbrechen" gewählt hat (oder nichts ausgewählt hat)
if [ $exit_status -ne 0 ] || [ ${#selected_id_list[@]} -eq 0 ]; then
    log "true" "false" "WARN" "Keine Workflows ausgewählt. Skript wird beendet."
    exit 1
fi

# Ab hier mit der Liste der IDs weiterarbeiten
log "true" "true" "INFO" "Verarbeite ${#selected_id_list[@]} ausgewählte Workflows:"

for github_workflow_id in "${selected_id_list[@]}"; do
        log "true" "true" "INFO" "Verarbeite jetzt ID: $github_workflow_id"
        # show_workflow_runs "$github_workflow_id"  # Optional: Limit ändern, z.B. 20
        # ...
        # ... 'while' 
        # ... angepasst, um mit der '$id' zu arbeiten
        # ... while loop anpassen  mit $id benutzt werden kann
        # while [[ "$current_date" < "$end_date" ]] || [[ "$current_date" == "$end_date" ]]; do
        # 	echo "Checking runs for $current_date"
        # 	runs=$(gh run list --created $current_date --json databaseId --jq '.[].databaseId')
        # 	if [[]]
        # 	if [[ -n "$runs" ]]; then
        # 		echo "Found runs for $current_date, deleting..."
        # 		echo "$runs" | xargs -I {} gh run delete {}
        # 	else
        # 		echo "No runs found for $current_date"
        # 	fi
        # 	current_date=$(date -j -v+1d -f "%Y-%m-%d" "$current_date" +%Y-%m-%d)
        # done
        # echo "Done deleting runs from $start_date to $end_date"
         show_workflow_runs "$github_workflow_id"
        
        selected_run_ids=()
        mapfile -t selected_run_ids < <(prompt_user_for_runs "$github_workflow_id")
        
        if [ ${#selected_run_ids[@]} -gt 0 ]; then
            for run_id in "${selected_run_ids[@]}"; do
                log "true" "true" "INFO" "Lösche Run ID: $run_id"
                gh run delete "$run_id"
                if [ $? -eq 0 ]; then
                    log "true" "true" "SUCCESS" "Run ID $run_id erfolgreich gelöscht."
                else
                    log "true" "true" "ERROR" "Fehler beim Löschen von Run ID $run_id."
                fi
            done
        else
            log "true" "true" "INFO" "Keine Runs zum Löschen ausgewählt für Workflow $github_workflow_id."
        fi
done
