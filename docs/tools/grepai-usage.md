# grepai - Semantische Code-Suche

## Übersicht

grepai ist ein Privacy-First CLI-Tool für semantische Code-Suche mittels Vektor-Embeddings. Im Gegensatz zu `grep` (exakter Text-Match) indiziert grepai die **Bedeutung** des Codes und ermöglicht natürlichsprachliche Suchen.

## Installation

```bash
curl -sSL https://raw.githubusercontent.com/yoanbernabeu/grepai/main/install.sh | sh
```

## Voraussetzungen

grepai benötigt einen Embedding-Provider:

| Provider | Typ | Setup |
|----------|-----|-------|
| **Ollama** (empfohlen) | Lokal, kostenlos | `brew install ollama && ollama pull nomic-embed-text` |
| **LM Studio** | Lokal | LM Studio starten mit Embedding-Modell |
| **OpenAI** | Cloud | `export OPENAI_API_KEY=sk-...` |

## Projekt-Setup

```bash
cd dein-projekt
grepai init          # Provider wählen (1=Ollama)
grepai watch         # Indexierung starten
```

## Verwendung

### Semantische Suche

```bash
# Natürlichsprachliche Suche
grepai search "error handling"
grepai search "authentication flow"
grepai search "database connection"

# Mit Optionen
grepai search "user login" -n 10       # Limit auf 10 Ergebnisse
grepai search "API endpoint" --json    # JSON-Output für AI-Agents
```

### Call-Graph Analyse

```bash
grepai trace callers "Login"              # Wer ruft Login() auf?
grepai trace callees "HandleRequest"      # Was ruft HandleRequest() auf?
grepai trace graph "ProcessOrder" --depth 3  # Kompletter Call-Graph
```

### Daemon-Management

```bash
grepai watch --background    # Im Hintergrund starten
grepai watch --status        # Status prüfen
grepai watch --stop          # Stoppen
```

## Claude Code Integration

### Als MCP-Server (Global)

grepai wurde bereits global als MCP-Server konfiguriert:

```bash
claude mcp add grepai --scope user -- grepai mcp-serve
```

Claude Code kann jetzt grepai-Tools direkt verwenden.

### Projekt-spezifische Integration

Im jeweiligen Projekt:

```bash
grepai agent-setup              # Fügt Anweisungen zu CLAUDE.md hinzu
grepai agent-setup --with-subagent  # Erstellt deep-explore Agent
```

## Konfiguration

Die Konfiguration liegt in `.grepai/config.yaml`:

```yaml
embedder:
  provider: ollama
  model: nomic-embed-text
  endpoint: http://localhost:11434
  dimensions: 768

store:
  backend: gob  # oder postgres, qdrant

search:
  boost:
    enabled: true  # Tests/Mocks niedriger gewichten
  hybrid:
    enabled: false  # Vector + Text kombinieren
```

## Tipps

- **Erster Start**: `grepai watch` kann bei großen Projekten einige Minuten dauern
- **Ressourcen**: Ollama benötigt ~300 MB RAM für das Embedding-Modell
- **Dateitypen**: .md Dateien werden niedriger gewichtet, Source-Code höher
- **Aktualisierung**: `grepai update` holt die neueste Version

## Ressourcen

- GitHub: https://github.com/yoanbernabeu/grepai
- Ollama: https://ollama.com
