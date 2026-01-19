#!/usr/bin/env python3
"""
Auto-translate common English phrases to German in .po files.
Only translates standard documentation phrases, not code or technical terms.
"""

import re
from pathlib import Path

# Common translations for documentation
TRANSLATIONS = {
    # Headings
    "Overview": "Überblick",
    "Description": "Beschreibung",
    "Structure": "Struktur",
    "Example": "Beispiel",
    "Examples": "Beispiele",
    "Usage": "Verwendung",
    "Configuration": "Konfiguration",
    "Installation": "Installation",
    "Requirements": "Anforderungen",
    "Parameters": "Parameter",
    "Returns": "Rückgabe",
    "Raises": "Auslöser",
    "Note": "Hinweis",
    "Warning": "Warnung",
    "Important": "Wichtig",
    "See also": "Siehe auch",
    "Further Information": "Weitere Informationen",
    "Best Practices": "Best Practices",
    "Common Patterns": "Gängige Muster",
    "Troubleshooting": "Fehlerbehebung",
    
    # Common phrases
    "This module provides": "Dieses Modul stellt bereit",
    "The following": "Die folgenden",
    "For more information": "Für weitere Informationen",
    "See the": "Siehe",
    "Available": "Verfügbar",
    "Optional": "Optional",
    "Required": "Erforderlich",
    "Default": "Standard",
    "Type": "Typ",
    "Field": "Feld",
    "Value": "Wert",
    "Name": "Name",
    "Path": "Pfad",
    "File": "Datei",
    "Directory": "Verzeichnis",
    "Module": "Modul",
    "Class": "Klasse",
    "Function": "Funktion",
    "Method": "Methode",
    "Attribute": "Attribut",
    "Property": "Eigenschaft",
    
    # Status
    "Success": "Erfolg",
    "Error": "Fehler",
    "Failed": "Fehlgeschlagen",
    "Completed": "Abgeschlossen",
    "Running": "Läuft",
    "Stopped": "Gestoppt",
    "Paused": "Pausiert",
}

def translate_po_file(po_file: Path):
    """Translate empty msgstr in a .po file."""
    content = po_file.read_text(encoding='utf-8')
    lines = content.split('\n')
    
    modified = False
    new_lines = []
    i = 0
    
    while i < len(lines):
        line = lines[i]
        
        # Look for msgid followed by empty msgstr
        if line.startswith('msgid "') and not line.startswith('msgid ""'):
            msgid_line = line
            msgid_text = line[7:-1]  # Extract text between quotes
            
            # Check next line for msgstr
            if i + 1 < len(lines) and lines[i + 1] == 'msgstr ""':
                # Check if we have a translation for this exact text
                if msgid_text in TRANSLATIONS:
                    translation = TRANSLATIONS[msgid_text]
                    new_lines.append(msgid_line)
                    new_lines.append(f'msgstr "{translation}"')
                    modified = True
                    i += 2
                    continue
        
        new_lines.append(line)
        i += 1
    
    if modified:
        po_file.write_text('\n'.join(new_lines), encoding='utf-8')
        return True
    return False

def main():
    docs_dir = Path(__file__).parent
    locale_dir = docs_dir / 'locale' / 'de' / 'LC_MESSAGES'
    
    print("🌍 Auto-translating common phrases in .po files...")
    print(f"📂 Locale directory: {locale_dir}")
    print(f"📝 Translation dictionary: {len(TRANSLATIONS)} entries\n")
    
    # Find all .po files
    po_files = list(locale_dir.glob('*.po')) + list(locale_dir.glob('*/*.po'))
    
    translated_count = 0
    for po_file in sorted(po_files):
        if translate_po_file(po_file):
            translated_count += 1
            rel_path = po_file.relative_to(locale_dir)
            print(f"✅ Translated: {rel_path}")
    
    print(f"\n✨ Auto-translated {translated_count} files")
    print(f"📊 Total files processed: {len(po_files)}")
    
    if translated_count > 0:
        print("\n⚠️  Note: This script only translates common headings and phrases.")
        print("   Full paragraph translations still need manual work.")
        print("   Run './build_multilingual.sh' to rebuild documentation.")

if __name__ == '__main__':
    main()
