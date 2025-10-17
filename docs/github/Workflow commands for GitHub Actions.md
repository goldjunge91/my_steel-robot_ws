# Workflow commands for GitHub Actions (Workflowbefehle für GitHub Actions)
Du kannst Workflow-Befehle verwenden, wenn du Shell-Befehle in einem Workflow oder im Code einer Aktion ausführst.
Informationen zu Workflow-Befehlen
Aktionen können mit dem Runner-Rechner kommunizieren, um Umgebungsvariablen zu setzen, Werte zur Verwendung in anderen Aktionen auszugeben, Debug-Meldungen zu den Ausgabeprotokollen zuzufügen und für andere Zwecke.

Die meisten Workflowbefehle verwenden den Befehl echo in einem spezifischen Format, während andere durch Schreiben in eine Datei aufgerufen werden. Weitere Informationen findest du unter Umgebungsdateien.

Beispiel für einen Workflowbefehl
Bash
echo "::workflow-command parameter1={data},parameter2={data}::{command value}"
Hinweis

Bei den Namen von Workflowbefehlen und -parametern wird nicht zwischen Groß- und Kleinschreibung unterschieden.

Warnung

Wenn du die Eingabeaufforderung verwendest, lasse doppelte Anführungszeichen (") bei Verwendung von Workflowbefehlen aus.

Workflow-Befehle verwenden, um auf Funktionen des Toolkits zuzugreifen
Die Aktionen/Toolkit enthält eine Reihe von Funktionen, die als Workflowbefehle ausgeführt werden können. Verwende die Syntax ::, um die Workflowbefehle in deiner YAML-Datei auszuführen. Diese Befehle werden dann über stdout an den Runner gesendet.

Statt beispielsweise Code zum Erstellen einer Fehleranmerkung wie unten zu verwenden:

JavaScript
core.error('Missing semicolon', {file: 'app.js', startLine: 1})
Beispiel: Erstellen einer Anmerkung zu einem Fehler
Du kannst den Befehl error in deinem Workflow verwenden, um dieselbe Fehleranmerkung zu erstellen:

YAML
      - name: Create annotation for build error
        run: echo "::error file=app.js,line=1::Missing semicolon"
Die folgende Tabelle zeigt, welche Toolkit-Funktionen innerhalb eines Workflows verfügbar sind:

Toolkit-Funktion	Äquivalenter Workflow-Befehl
core.addPath	Barrierefrei mithilfe der Umgebungsdatei GITHUB_PATH
core.debug	debug
core.notice	notice
core.error	error
core.endGroup	endgroup
core.exportVariable	Barrierefrei mithilfe der Umgebungsdatei GITHUB_ENV
core.getInput	Barrierefrei mithilfe der Umgebungsvariablen INPUT_{NAME}
core.getState	Barrierefrei mithilfe der Umgebungsvariablen STATE_{NAME}
core.isDebug	Barrierefrei mithilfe der Umgebungsvariablen RUNNER_DEBUG
core.summary	Barrierefrei mithilfe der Umgebungsdatei GITHUB_STEP_SUMMARY
core.saveState	Barrierefrei mithilfe der Umgebungsdatei GITHUB_STATE
core.setCommandEcho	echo
core.setFailed	Wird als Abkürzung für ::error und exit 1 verwendet
core.setOutput	Barrierefrei mithilfe der Umgebungsdatei GITHUB_OUTPUT
core.setSecret	add-mask
core.startGroup	group
core.warning	warning
Festlegen einer Debugmeldung
Gibt eine Debugging-Meldung im Protokoll aus. Du musst einen Geheimschlüssel ACTIONS_STEP_DEBUG mit dem Wert true erstellen, um die Debugmeldungen anzuzeigen, die in diesem Befehl im Protokoll festgelegt sind. Weitere Informationen finden Sie unter Aktivieren der Debugprotokollierung.

Text
::debug::{message}
Beispiel: Festlegen einer Debugmeldung
Bash
echo "::debug::Set the Octocat variable"
Festlegen einer Benachrichtigung
Erstellt eine Benachrichtigung und fügt diese in das Protokoll ein. Diese Nachricht erstellt eine Anmerkung, die die Nachricht einer bestimmten Datei in deinem Repository zuordnen kann. Optional kann deine Nachricht eine Position innerhalb der Datei angeben.

Text
::notice file={name},line={line},endLine={endLine},title={title}::{message}
Parameter	Wert	Erforderlich	Standard
title	Benutzerdefinierter Titel	No	Keine
file	Filename	No	.github
col	Spaltennummer ab 1	No	Keine
endColumn	Endspaltennummer	No	Keine
line	Zeilennummer ab 1	No	1
endLine	Endzeilennummer	No	1
Beispiel: Festlegen einer Benachrichtigung
Bash
echo "::notice file=app.js,line=1,col=5,endColumn=7::Missing semicolon"
Festlegen einer Warnmeldung
Erstellt eine Warnmeldung und fügt die Mitteilung in das Protokoll ein. Diese Nachricht erstellt eine Anmerkung, die die Nachricht einer bestimmten Datei in deinem Repository zuordnen kann. Optional kann deine Nachricht eine Position innerhalb der Datei angeben.

Text
::warning file={name},line={line},endLine={endLine},title={title}::{message}
Parameter	Wert	Erforderlich	Standard
title	Benutzerdefinierter Titel	No	Keine
file	Filename	No	.github
col	Spaltennummer ab 1	No	Keine
endColumn	Endspaltennummer	No	Keine
line	Zeilennummer ab 1	No	1
endLine	Endzeilennummer	No	1
Beispiel: Festlegen einer Warnmeldung
Bash
echo "::warning file=app.js,line=1,col=5,endColumn=7,title=YOUR-TITLE::Missing semicolon"
Festlegen einer Fehlermeldung
Erstellt eine Fehlermeldung und fügt die Mitteilung in das Protokoll ein. Diese Nachricht erstellt eine Anmerkung, die die Nachricht einer bestimmten Datei in deinem Repository zuordnen kann. Optional kann deine Nachricht eine Position innerhalb der Datei angeben.

Text
::error file={name},line={line},endLine={endLine},title={title}::{message}
Parameter	Wert	Erforderlich	Standard
title	Benutzerdefinierter Titel	No	Keine
file	Filename	No	.github
col	Spaltennummer ab 1	No	Keine
endColumn	Endspaltennummer	No	Keine
line	Zeilennummer ab 1	No	1
endLine	Endzeilennummer	No	1
Beispiel: Festlegen einer Fehlermeldung
Bash
echo "::error file=app.js,line=1,col=5,endColumn=7,title=YOUR-TITLE::Missing semicolon"
Gruppieren von Protokollzeilen
Erstellt eine erweiterbare Gruppe im Protokoll. Verwende den Befehl group, um eine Gruppe zu erstellen und title festzulegen. Alles, was du im Protokoll zwischen den group Befehlen endgroup einfügst, wird in einem erweiterbaren Eintrag im Protokoll geschachtelt.

Text
::group::{title}
::endgroup::
Beispiel: Gruppieren von Protokollzeilen
YAML
jobs:
  bash-example:
    runs-on: ubuntu-latest
    steps:
      - name: Group of log lines
        run: |
            echo "::group::My title"
            echo "Inside group"
            echo "::endgroup::"
Screenshot des Protokolls für den Workflowschritt. Die zweite Zeile „My title“ ist eine erweiterte Gruppe. Die nächste Zeile, „In der Gruppe“, wird unten eingerückt.

Maskieren eines Werts in einem Protokoll
Text
::add-mask::{value}
Das Maskieren eines Werts verhindert, dass ein String oder eine Variable im Protokoll ausgegeben werden. Jedes maskierte Wort, getrennt durch Leerzeichen, wird durch das Zeichen * ersetzt. Du kannst eine Umgebungsvariable oder Zeichenfolge für den Wert value der Maske verwenden. Wenn du einen Wert maskierst, wird er als geheim behandelt und auf dem Runner bearbeitet. Wenn du beispielsweise einen Wert maskierst, kannst du diesen Wert nicht als Ausgabe festlegen.

Beispiel: Maskieren einer Zeichenfolge
Wenn du im Protokoll "Mona The Octocat" einfügst, wird "***" angezeigt.

Bash
echo "::add-mask::Mona The Octocat"
Warnung

Stelle sicher, dass du das Geheimnis mit „add-mask“ registrierst, bevor es in den Buildprotokollen ausgegeben oder in anderen Workflowbefehlen verwendet wird.

Beispiel: Maskieren einer Umgebungsvariablen
Wenn du die Variable MY_NAME oder den Wert "Mona The Octocat" im Protokoll einfügst, wird "***" anstelle von "Mona The Octocat" angezeigt.

YAML
jobs:
  bash-example:
    runs-on: ubuntu-latest
    env:
      MY_NAME: "Mona The Octocat"
    steps:
      - name: bash-version
        run: echo "::add-mask::$MY_NAME"
Beispiel: Maskieren einer generierten Ausgabe innerhalb eines einzelnen Auftrags
Wenn du dein Geheimnis nicht von einem Auftrag an einen anderen Auftrag übergeben musst, hast du folgende Möglichkeiten:

Generiere das Geheimnis (ohne es auszugeben).
Maskiere es mit add-mask.
Verwende GITHUB_OUTPUT, um das Geheimnis für andere Schritte innerhalb des Auftrags verfügbar zu machen.
YAML
on: push
jobs:
  generate-a-secret-output:
    runs-on: ubuntu-latest
    steps:
      - id: sets-a-secret
        name: Generate, mask, and output a secret
        run: |
          the_secret=$((RANDOM))
          echo "::add-mask::$the_secret"
          echo "secret-number=$the_secret" >> "$GITHUB_OUTPUT"
      - name: Use that secret output (protected by a mask)
        run: |
          echo "the secret number is ${{ steps.sets-a-secret.outputs.secret-number }}"
Beispiel: Maskieren und Übergeben eines Geheimnisses zwischen Aufträgen oder Workflows
Wenn du ein maskiertes Geheimnis zwischen Aufträgen oder Workflows übergeben möchtest, solltest du das Geheimnis in einem Speicher speichern und dann im nachfolgenden Auftrag oder Workflow abrufen.

Einrichten
Richte einen Geheimnisspeicher ein, um das Geheimnis zu speichern, das du während deines Workflows generierst. Beispiel: Tresor.
Generiere einen Schlüssel zum Lesen und Schreiben in diesem Geheimspeicher. Speichere den Schlüssel als Repositorygeheimnis. Im folgenden Beispielworkflow lautet der Geheimnisname SECRET_STORE_CREDENTIALS. Weitere Informationen finden Sie unter Verwenden von Geheimnissen in GitHub-Aktionen.
Workflow
Hinweis

Dieser Workflow verwendet einen imaginären Geheimnisspeicher (secret-store), der die imaginären Befehle store-secret und retrieve-secret enthält. some/secret-store@ 27b31702a0e7fc50959f5ad993c78deac1bdfc29 ist eine imaginäre Aktion, die die Anwendung secret-store installiert und für die Verbindung mit instance mit credentials konfiguriert.

YAML
on: push

jobs:
  secret-generator:
    runs-on: ubuntu-latest
    outputs:
      handle: ${{ steps.generate-secret.outputs.handle }}
    steps:
    - uses: some/secret-store@27b31702a0e7fc50959f5ad993c78deac1bdfc29
      with:
        credentials: ${{ secrets.SECRET_STORE_CREDENTIALS }}
        instance: ${{ secrets.SECRET_STORE_INSTANCE }}
    - name: generate secret
      id: generate-secret
      shell: bash
      run: |
        GENERATED_SECRET=$((RANDOM))
        echo "::add-mask::$GENERATED_SECRET"
        SECRET_HANDLE=$(secret-store store-secret "$GENERATED_SECRET")
        echo "handle=$SECRET_HANDLE" >> "$GITHUB_OUTPUT"
  secret-consumer:
    runs-on: macos-latest
    needs: secret-generator
    steps:
    - uses: some/secret-store@27b31702a0e7fc50959f5ad993c78deac1bdfc29
      with:
        credentials: ${{ secrets.SECRET_STORE_CREDENTIALS }}
        instance: ${{ secrets.SECRET_STORE_INSTANCE }}
    - name: use secret
      shell: bash
      run: |
        SECRET_HANDLE="${{ needs.secret-generator.outputs.handle }}"
        RETRIEVED_SECRET=$(secret-store retrieve-secret "$SECRET_HANDLE")
        echo "::add-mask::$RETRIEVED_SECRET"
        echo "We retrieved our masked secret: $RETRIEVED_SECRET"
Beenden und Starten von Workflowbefehlen
Beendet die Verarbeitung von Workflowbefehlen. Mit diesem speziellen Befehl kannst du alles protokollieren, ohne versehentlich einen Workflowbefehl auszuführen. Du kannst beispielsweise die Protokollierung anhalten, um ein vollständiges Skript mit Kommentaren auszugeben.

Text
::stop-commands::{endtoken}
Um die Verarbeitung von Workflowbefehlen zu beenden, übergib ein eindeutiges Token an stop-commands. Um die Verarbeitung von Workflowbefehlen fortzusetzen, übergib dasselbe Token, das du zum Beenden von Workflowbefehlen verwendet hast.

Warnung

Stelle sicher, dass das verwendete Token zufällig generiert und für jede Ausführung eindeutig ist.

Text
::{endtoken}::
Beispiel: Beenden und Starten von Workflowbefehlen
YAML
jobs:
  workflow-command-job:
    runs-on: ubuntu-latest
    steps:
      - name: Disable workflow commands
        run: |
          echo '::warning:: This is a warning message, to demonstrate that commands are being processed.'
          stopMarker=$(uuidgen)
          echo "::stop-commands::$stopMarker"
          echo '::warning:: This will NOT be rendered as a warning, because stop-commands has been invoked.'
          echo "::$stopMarker::"
          echo '::warning:: This is a warning again, because stop-commands has been turned off.'
Werte an die „Pre-“ (Vor-) und „Post-“ (Nach-)Aktionen senden
Sie können mit den Aktionen pre: oder post: ihres Workflows Umgebungsvariablen zur Freigabe erstellen, indem Sie in die Datei unter GITHUB_STATE schreiben. Du kannst beispielsweise eine Datei mit der Aktion pre: erstellen, den Dateispeicherort an die Aktion main: übergeben und dann die Aktion post: verwenden, um die Datei zu löschen. Alternativ kannst du eine Datei mit der Aktion main: erstellen, den Dateispeicherort an die Aktion post: übergeben und auch die Aktion post: verwenden, um die Datei zu löschen.

Wenn Sie mehrere pre:- oder post:-Aktionen haben, können Sie nur auf den gespeicherten Wert in der Aktion zugreifen, in der er in GITHUB_STATE geschrieben wurde. Weitere Informationen zur post:-Aktion findest du unter Referenz zur Metadatensyntax.

Die GITHUB_STATE-Datei ist nur in einer Aktion verfügbar. Der gespeicherte Wert wird als Umgebungswert mit dem Präfix STATE_ gespeichert.

In diesem Beispiel wird JavaScript zum Schreiben in die GITHUB_STATE-Datei verwendet. Die resultierende Umgebungsvariable wird STATE_processID genannt und hat den Wert 12345:

JavaScript
import * as fs from 'fs'
import * as os from 'os'

fs.appendFileSync(process.env.GITHUB_STATE, `processID=12345${os.EOL}`, {
  encoding: 'utf8'
})
Die Variable STATE_processID ist dann ausschließlich für das unter der Aktion main ausgeführte Bereinigungsskript verfügbar. Dieses Beispiel läuft in main und verwendet JavaScript, um den Wert anzuzeigen, der der Umgebungsvariable STATE_processID zugewiesen wurde:

JavaScript
console.log("The running PID from the main action is: " + process.env.STATE_processID);
Umgebungsdateien
Während der Ausführung eines Workflows generiert der Runner temporäre Dateien, die zum Ausführen bestimmter Aktionen verwendet werden können. Auf den Pfad zu diesen Dateien kann mithilfe der Standardumgebungsvariablen von GitHub zugegriffen und bearbeitet werden. Weitere Informationen findest du unter Variablenverweis. Du musst UTF-8-Codierung verwenden, wenn du in diese Dateien schreibst, um die ordnungsgemäße Verarbeitung der Befehle sicherzustellen. Mehrere Befehle können in dieselbe Datei geschrieben werden, getrennt durch Zeilenvorschubzeichen. Um Umgebungsvariablen in einer GitHub-Aktion zu verwenden, erstellen oder ändern Sie .env-Dateien mit bestimmten GitHub-Aktionsbefehlen.

Gehen Sie dazu wie folgt vor:

YAML
name: Example Workflow for Environment Files

on: push

jobs:
  set_and_use_env_vars:
    runs-on: ubuntu-latest
    steps:
      - name: Set environment variable
        run: echo "MY_ENV_VAR=myValue" >> $GITHUB_ENV

      - name: Use environment variable
        run: |
          echo "The value of MY_ENV_VAR is $MY_ENV_VAR"

Ein weiteres Beispiel wäre die Verwendung, um Metadaten wie Buildzeitstempel, Commit-SHAs oder Artefaktnamen zu speichern:

YAML
steps:
  - name: Store build timestamp
    run: echo "BUILD_TIME=$(date +'%T')" >> $GITHUB_ENV

  - name: Deploy using stored timestamp
    run: echo "Deploying at $BUILD_TIME"
Festlegen einer Umgebungsvariablen
Hinweis

Um Probleme zu vermeiden, wird empfohlen, bei Umgebungsvariablen unabhängig vom Verhalten des verwendeten Betriebssystems und der verwendeten Shell die Groß-/Kleinschreibung zu beachten.

Bash
echo "{environment_variable_name}={value}" >> "$GITHUB_ENV"
Du kannst eine Umgebungsvariable für alle nachfolgenden Schritte in einem Workflowauftrag verfügbar machen, indem du die Umgebungsvariable definierst oder aktualisierst und diese in die Umgebungsdatei GITHUB_ENV schreibst. Der Schritt, der die Umgebungsvariable erstellt oder aktualisiert, hat keinen Zugriff auf den neuen Wert, aber alle nachfolgenden Schritte in einem Auftrag haben Zugriff.

Du kannst den Wert der Standardumgebungsvariablen namens GITHUB_* und RUNNER_* nicht überschreiben. Derzeit kannst du den Wert der Variablen CI überschreiben. Es ist jedoch nicht garantiert, dass dies immer möglich sein wird. Weitere Informationen zu den Standardumgebungsvariablen findest du unter Speichern von Informationen in Variablen.

Hinweis

Aufgrund von Sicherheitseinschränkungen kann GITHUB_ENV nicht zum Festlegen der Umgebungsvariable NODE_OPTIONS verwendet werden.

Beispiel für das Schreiben einer Umgebungsvariablen in GITHUB_ENV
YAML
steps:
  - name: Set the value
    id: step_one
    run: |
      echo "action_state=yellow" >> "$GITHUB_ENV"
  - name: Use the value
    id: step_two
    run: |
      printf '%s\n' "$action_state" # This will output 'yellow'
Mehrzeilige Zeichenfolgen
Bei mehrzeiligen Zeichenfolgen kannst du ein Trennzeichen mit der folgenden Syntax verwenden.

Text
{name}<<{delimiter}
{value}
{delimiter}
Warnung

Stelle sicher, dass das verwendete Trennzeichen nicht in einer eigenen Zeile innerhalb des Wertes vorkommt. Wenn der Wert völlig zufällig ist, sollten Sie dieses Format nicht verwenden. Schreiben Sie den Wert stattdessen in eine Datei.

Beispiel für eine mehrzeilige Zeichenfolge
In diesem Beispiel wird EOF als Trennzeichen verwendet und die Umgebungsvariable JSON_RESPONSE auf den Wert der Antwort curl festgelegt.

YAML
steps:
  - name: Set the value in bash
    id: step_one
    run: |
      {
        echo 'JSON_RESPONSE<<EOF'
        curl https://example.com
        echo EOF
      } >> "$GITHUB_ENV"
Festlegen eines Ausgabeparameters
Legt den Ausgabeparameter eines Schritts fest. Beachte, dass für den Schritt eine id definiert werden muss, um später den Ausgabewert abzurufen. Du kannst mehrzeilige Ausgabewerte auf dieselbe Weise festlegen wie im Abschnitt Mehrzeilige Zeichenfolgen, um mehrzeilige Umgebungsvariablen zu definieren.

Bash
echo "{name}={value}" >> "$GITHUB_OUTPUT"
Beispiel für das Festlegen eines Ausgabeparameters
Dieses Beispiel zeigt, wie man den Ausgabeparameter SELECTED_COLOR festlegt und ihn später abruft:

YAML
      - name: Set color
        id: color-selector
        run: echo "SELECTED_COLOR=green" >> "$GITHUB_OUTPUT"
      - name: Get color
        env:
          SELECTED_COLOR: ${{ steps.color-selector.outputs.SELECTED_COLOR }}
        run: echo "The selected color is $SELECTED_COLOR"
Hinzufügen einer Auftragszusammenfassung
Bash
echo "{markdown content}" >> $GITHUB_STEP_SUMMARY
Du kannst ein benutzerdefiniertes Markdown für die einzelnen Aufträge festlegen, sodass sie auf der Zusammenfassungsseite einer Workflowausführung angezeigt werden. Du kannst Auftragszusammenfassungen verwenden, um eindeutige Inhalte (z. B. Zusammenfassungen von Testergebnissen) anzuzeigen und zu gruppieren, damit Personen, die das Ergebnis einer Workflowausführung anzeigen, nicht in die Protokolle wechseln müssen, um wichtige ausführungsbezogene Informationen (z. B. Fehler) anzuzeigen.

Auftragszusammenfassungen unterstützen GitHub-Markdown, und du kannst der GITHUB_STEP_SUMMARY-Umgebungsdatei eigenen Markdowninhalt für einen Schritt hinzufügen. GITHUB_STEP_SUMMARY ist für jeden Schritt in einem Auftrag eindeutig. Weitere Informationen zu der Pro-Schritt-Datei, auf die GITHUB_STEP_SUMMARY verweist, findest du unter Umgebungsdateien.

Bei Abschluss eines Auftrags werden die Zusammenfassungen für alle Schritte in einem Auftrag in einer einzelnen Auftragszusammenfassung gruppiert und auf der Zusammenfassungsseite der Workflowausführung angezeigt. Wenn für mehrere Aufträge Zusammenfassungen generiert werden, werden diese nach Auftragsabschlusszeit sortiert.

Beispiel für das Hinzufügen einer Auftragszusammenfassung
Bash
echo "### Hello world! :rocket:" >> $GITHUB_STEP_SUMMARY
Screenshot der Zusammenfassungsseite einer Workflowausführung. Unter „Beispielzusammenfassung“ befinden sich „Hello world!“ und ein Raketen-Emoji.

Mehrzeiliger Markdowninhalt
Für mehrzeiligen Markdowninhalt kannst du mit >> kontinuierlich Inhalte für den aktuellen Schritt anfügen. Bei jedem Anfügevorgang wird automatisch ein Zeilenvorschubzeichen hinzugefügt.

Beispiel für mehrzeiligen Markdowninhalt
- name: Generate list using Markdown
  run: |
    echo "This is the lead in sentence for the list" >> $GITHUB_STEP_SUMMARY
    echo "" >> $GITHUB_STEP_SUMMARY # this is a blank line
    echo "- Lets add a bullet point" >> $GITHUB_STEP_SUMMARY
    echo "- Lets add a second bullet point" >> $GITHUB_STEP_SUMMARY
    echo "- How about a third one?" >> $GITHUB_STEP_SUMMARY
Überschreiben von Auftragszusammenfassungen
Zum Löschen aller Inhalte für den aktuellen Schritt können Sie > verwenden, um alle zuvor hinzugefügten Inhalte in der Bash zu überschreiben, oder -Append in der PowerShell entfernen

Beispiel für das Überschreiben von Auftragszusammenfassungen
- name: Overwrite Markdown
  run: |
    echo "Adding some Markdown content" >> $GITHUB_STEP_SUMMARY
    echo "There was an error, we need to clear the previous Markdown with some new content." > $GITHUB_STEP_SUMMARY
Entfernen von Auftragszusammenfassungen
Zum vollständigen Entfernen einer Zusammenfassung für den aktuellen Schritt kannst du die Datei löschen, auf die GITHUB_STEP_SUMMARY verweist.

Beispiel für das Entfernen von Auftragszusammenfassungen
- name: Delete all summary content
  run: |
    echo "Adding Markdown content that we want to remove before the step ends" >> $GITHUB_STEP_SUMMARY
    rm $GITHUB_STEP_SUMMARY
Nachdem ein Schritt abgeschlossen wurde, werden Auftragszusammenfassungen hochgeladen, und zuvor hochgeladene Markdowninhalte können durch nachfolgende Schritte nicht geändert werden. Alle Geheimnisse, die versehentlich hinzugefügt wurden, werden von Zusammenfassungen automatisch maskiert. Wenn eine Auftragszusammenfassung vertrauliche Informationen enthält, die gelöscht werden müssen, kannst du die gesamte Workflowausführung löschen, um alle zugehörigen Auftragszusammenfassungen zu entfernen. Weitere Informationen findest du unter Eine Workflowausführung löschen.

Schrittisolierung und Grenzwerte
Auftragszusammenfassungen werden zwischen Schritten isoliert und jeder Schritt ist auf eine maximale Größe von 1 MiB beschränkt. Die Isolation wird zwischen Schritten erzwungen, damit das Markdownrendering für nachfolgende Schritte nicht durch potenziell fehlerhaftes Markdown unterbrochen werden kann. Wenn für einen Schritt Inhalte von mehr als 1 MiB hinzugefügt werden, schlägt der Upload für den Schritt fehl, und es wird eine Fehleranmerkung erstellt. Uploadfehler bei Auftragszusammenfassungen wirken sich nicht auf den Gesamtstatus eines Schritts oder Auftrags aus. Pro Auftrag werden maximal 20 Auftragszusammenfassungen aus Schritten angezeigt.

Hinzufügen eines Systempfads
Stellt ein Verzeichnis der Systemvariable PATH vor und stellt es automatisch für alle nachfolgenden Aktionen im aktuellen Auftrag zur Verfügung. Die aktuell ausgeführte Aktion kann nicht auf die aktualisierte Pfadvariable zugreifen. Um die aktuell definierten Pfade für deinen Auftrag anzuzeigen, kannst du in einem Schritt oder einer Aktion echo "$PATH" verwenden.

Beispiel für das Hinzufügen eines Systempfads
In diesem Beispiel wird veranschaulicht, wie du das Benutzerverzeichnis $HOME/.local/bin zu PATH hinzufügst:

Bash
echo "$HOME/.local/bin" >> "$GITHUB_PATH"
