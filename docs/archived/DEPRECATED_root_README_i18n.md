# Multilingual Documentation (English/German)

This directory contains the multilingual documentation for VYRA Base Python Library, supporting both English and German.

## 🌍 Supported Languages

- **English** (`en`) - Default language
- **Deutsch** (`de`) - German translation

## 📋 Quick Start

### Build Locally

Run the build script to generate both language versions:

```bash
cd docs
./build_multilingual.sh
```

This will:
1. Extract translatable strings (POT files)
2. Update PO translation files
3. Apply automatic translations
4. Build English documentation
5. Build German documentation
6. Create an index page with language selection

### View Locally

After building, start a local web server:

```bash
python -m http.server -d _build/html 8000
```

Then open: http://localhost:8000

## 🔧 Manual Build

### Build English Version

```bash
sphinx-build -b html -D language=en . _build/html/en
```

### Build German Version

```bash
sphinx-build -b html -D language=de . _build/html/de
```

## 📝 Translation Workflow

### 1. Extract Translatable Strings

Generate POT (Portable Object Template) files:

```bash
sphinx-build -b gettext . _build/gettext
```

### 2. Update PO Files

Update PO (Portable Object) files for both languages:

```bash
sphinx-intl update -p _build/gettext -l de -l en
```

This creates/updates:
- `locale/de/LC_MESSAGES/*.po` - German translations

### 3. Translate Strings

Edit PO files directly in `locale/de/LC_MESSAGES/`:

```po
#: ../../index.rst:12
msgid "Getting Started"
msgstr "Erste Schritte"  # Add/update German translation here
```

**Important**: Only edit `msgstr`, never `msgid`!

### 4. Build Documentation

After translating, rebuild the documentation:

```bash
./build_multilingual.sh
```

Or build specific languages:

```bash
# English only
sphinx-build -b html -D language=en . _build/html/en

# German only
sphinx-build -b html -D language=de . _build/html/de
```

## 🤖 Continuous Integration

GitHub Actions automatically builds and deploys both language versions on push to `main`:

- **Workflow**: `.github/workflows/docs.yml`
- **Triggers**: Push to `main` or `develop`, Pull Requests
- **Output**: 
  - English: `https://<username>.github.io/<repo>/en/`
  - German: `https://<username>.github.io/<repo>/de/`
  - Redirect: `https://<username>.github.io/<repo>/` (auto-detects browser language)

## 📂 File Structure

```
docs/
├── conf.py                    # Sphinx configuration (gettext enabled)
├── build_multilingual.sh      # Local build script
├── locale/
│   └── de/LC_MESSAGES/        # German translations (*.po)
├── _build/
│   ├── gettext/               # Extracted POT files
│   └── html/
│       ├── en/                # Built English docs
│       ├── de/                # Built German docs
│       └── index.html         # Language selection redirect
└── _templates/
    ├── language_switcher.html # Language switcher widget
    └── page.html              # Page template with switcher
```

## 🎨 Language Switcher

The documentation includes a language switcher in the header (dropdown menu with flags):

- Automatically detects current language from URL
- Switches between `/en/` and `/de/` paths
- Preserves current page when switching languages

Template location: `_templates/language_switcher.html`

## ⚙️ Configuration

Key configuration in `conf.py`:

```python
# Internationalization
locale_dirs = ['locale/']           # Path to translation files
gettext_compact = False              # Separate POT file per document
gettext_uuid = True                  # Add UUID for tracking
gettext_allow_fuzzy_translations = True  # Include fuzzy translations
language = 'en'                      # Default language
```

## 🔍 Troubleshooting

### Warnings about duplicate cross-references

These are expected and don't affect the build. They occur because classes are exported at multiple levels.

### Warnings about `top_of_page_button`

This is a deprecated Furo theme option. It's been removed from config.

### Missing translations

1. Check if POT files were generated: `_build/gettext/*.pot`
2. Verify PO files exist: `locale/de/LC_MESSAGES/*.po`
3. Ensure `msgstr` is filled in PO files
4. Rebuild with `sphinx-build`

### Language switcher not working

1. Verify template files exist in `_templates/`
2. Check JavaScript console for errors
3. Ensure URLs match pattern: `/en/...` and `/de/...`

## 📚 Adding More Languages

To add a new language (e.g., French):

```bash
# 1. Update PO files for new language
sphinx-intl update -p _build/gettext -l fr

# 2. Translate in locale/fr/LC_MESSAGES/*.po

# 3. Build French version
sphinx-build -b html -D language=fr . _build/html/fr

# 4. Update language switcher template
# Add French option in _templates/language_switcher.html

# 5. Update GitHub Actions workflow
# Add French build step in .github/workflows/docs.yml
```

## 🛠️ Tools Used

- **Sphinx**: Documentation generator
- **sphinx-intl**: Internationalization utilities
- **Babel**: Translation catalog management
- **Furo**: Documentation theme
- **gettext**: GNU translation system

## 📖 Resources

- [Sphinx i18n Guide](https://www.sphinx-doc.org/en/master/usage/advanced/intl.html)
- [sphinx-intl Documentation](https://sphinx-intl.readthedocs.io/)
- [Babel Documentation](http://babel.pocoo.org/)
- [Furo Theme](https://pradyunsg.me/furo/)

## ✨ Best Practices

1. **Keep translations up-to-date**: Re-run `sphinx-intl update` after RST changes
2. **Review automatic translations**: Not all terms should be translated (e.g., code identifiers)
3. **Use consistent terminology**: Maintain a translation glossary
4. **Test both languages**: Always build and review both EN and DE versions
5. **Update CHANGELOG**: Document translation changes in version notes

---

**Questions?** Open an issue in the repository!
