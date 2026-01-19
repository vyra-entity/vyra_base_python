# Documentation Structure - VYRA Base Python

## ✅ Current State

The VYRA documentation is now properly structured with:

### 1. Source Documentation (English)
- All `.rst` files in `docs/` are written **directly in English**
- No Python scripts containing English text
- Clean, maintainable documentation source

### 2. German Translations
- Generated from English source via **gettext** (`.pot` → `.po` → `.mo`)
- Located in `locale/de/LC_MESSAGES/*.po`
- Translations managed through standard i18n workflow

### 3. Build Process
```bash
# Extract translatable strings
sphinx-build -b gettext . _build/gettext

# Update German translations
sphinx-intl update -p _build/gettext -l de

# Build English (default)
sphinx-build -b html . _build/html

# Build German
sphinx-build -b html -D language=de . _build/html/de

# Or build both at once
./build_multilingual.sh
```

## 📁 Directory Structure

```
docs/
├── *.rst                      # English source documentation
├── com/
│   ├── overview.rst           # English
│   ├── ros2_communication.rst # English
│   ├── ipc_communication.rst  # English
│   └── feeders.rst            # English
├── core/
│   ├── entity.rst             # English
│   ├── parameter.rst          # English
│   └── volatile.rst           # English
├── locale/
│   └── de/LC_MESSAGES/        # German translations (.po files)
├── _build/
│   ├── gettext/               # Extracted .pot files
│   └── html/
│       ├── en/                # Built English docs
│       └── de/                # Built German docs
├── build_multilingual.sh      # Build script
└── README_i18n.md             # i18n documentation
```

## 🔧 Maintenance Workflow

### Adding New Documentation
1. Write documentation in English (`.rst` files)
2. Extract translatable strings: `sphinx-build -b gettext . _build/gettext`
3. Update German PO files: `sphinx-intl update -p _build/gettext -l de`
4. Translate strings in `locale/de/LC_MESSAGES/*.po`
5. Build: `./build_multilingual.sh`

### Updating Existing Documentation
1. Edit `.rst` files (always in English)
2. Re-extract strings: `sphinx-build -b gettext . _build/gettext`
3. Update PO files: `sphinx-intl update -p _build/gettext -l de`
4. Update translations in PO files
5. Rebuild

## 🚫 Removed Files

The following files have been removed as they're no longer needed:
- `translate_all.py` - Not needed (RST files are already in English)
- `translate_comprehensive.py` - Not needed (manual PO translation)
- `translate_po_files.py` - Not needed (manual PO translation)
- `locale/en/` - Not needed (English is the source)
- `IMPLEMENTATION_SUMMARY.md` - Temporary documentation
- `TRANSLATION_COMPLETE.md` - Temporary documentation

## ✅ Best Practices

1. **Always write documentation in English** - No translation scripts needed
2. **Use gettext for German translations** - Standard i18n approach
3. **Manual PO file editing** - Better quality than automated translation
4. **Version control PO files** - Track translation changes
5. **Test both languages** - Build both EN and DE before committing

## 📊 Build Status

- ✅ English build: Clean (0 warnings)
- ✅ German build: Clean (0 warnings)
- ✅ Multilingual infrastructure: Working
- ✅ Language switcher: Functional

---

**Last Updated**: 2026-01-19
**Status**: Production Ready
