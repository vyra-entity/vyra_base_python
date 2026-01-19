# Multilingual Documentation Guide

## Overview

The VYRA Base Python documentation supports multiple languages using Sphinx's `gettext` internationalization system. Currently supported languages:

- 🇬🇧 **English** (default): `docs/_build/html/en/`
- 🇩🇪 **German**: `docs/_build/html/de/`

## Quick Start

Build all languages with a single command:

```bash
cd /home/holgder/VYRA/vyra_base_python/docs
./build_multilingual.sh
```

This will:
1. Extract translatable strings to `.pot` files
2. Update German `.po` translation files
3. Build English HTML documentation
4. Build German HTML documentation
5. Create language selector index page

## File Structure

```
docs/
├── _build/
│   ├── gettext/              # Generated .pot translation templates
│   └── html/
│       ├── index.html        # Language selector redirect
│       ├── en/               # English documentation
│       └── de/               # German documentation
├── locale/
│   └── de/
│       └── LC_MESSAGES/
│           ├── index.po      # Main page translations
│           ├── core.po       # Core components translations
│           ├── helper.po     # Helper functions translations
│           ├── interfaces_guide.po  # Interface guide translations
│           └── ...           # Other translation files
├── _templates/
│   └── sidebar/
│       └── language.html     # Language switcher template
├── conf.py                   # Sphinx configuration with i18n settings
└── build_multilingual.sh     # Build script for all languages
```

## Translation Workflow

### 1. Extract New Translatable Strings

After editing `.rst` files, extract updated strings:

```bash
cd docs
sphinx-build -b gettext . _build/gettext
```

This creates/updates `.pot` files in `_build/gettext/`.

### 2. Update Translation Files

Update German `.po` files with new strings:

```bash
sphinx-intl update -p _build/gettext -l de
```

This merges new strings from `.pot` files into `.po` files.

### 3. Translate Strings

Edit `.po` files in `locale/de/LC_MESSAGES/`:

```po
# Before translation
msgid "Core Components"
msgstr ""

# After translation
msgid "Core Components"
msgstr "Kern-Komponenten"
```

**Important**: Only edit `msgstr` (translation), never `msgid` (source English text).

### 4. Build Documentation

Build German documentation:

```bash
sphinx-build -D language=de -b html . _build/html/de
```

Or use the automated script:

```bash
./build_multilingual.sh
```

## Language Switcher

The language switcher appears in the sidebar of every page. Configuration in `conf.py`:

```python
html_context = {
    'languages': [
        ('en', 'English', '../en/'),
        ('de', 'Deutsch', '../de/'),
    ],
    'current_language': language,
}

html_sidebars = {
    '**': [
        'sidebar/language.html',  # Custom language switcher template
        'sidebar/scroll-start.html',
        'sidebar/brand.html',
        'sidebar/search.html',
        'sidebar/navigation.html',
        'sidebar/scroll-end.html',
    ]
}
```

## GitHub Pages Deployment

### GitHub Actions Workflow

The `.github/workflows/build-sphinx.yml` workflow automatically:

1. Builds both English and German documentation
2. Deploys to `gh-pages` branch
3. Makes documentation available at: `https://<username>.github.io/<repo>/`

### Manual Deployment

```bash
# Build documentation
./build_multilingual.sh

# Commit to gh-pages branch
cd _build/html
git init
git add .
git commit -m "Deploy documentation"
git push -f origin main:gh-pages
```

## Adding a New Language

### 1. Create New Locale

```bash
sphinx-intl update -p _build/gettext -l fr  # Example: French
```

### 2. Update conf.py

Add the new language to `html_context`:

```python
html_context = {
    'languages': [
        ('en', 'English', '../en/'),
        ('de', 'Deutsch', '../de/'),
        ('fr', 'Français', '../fr/'),  # New language
    ],
    'current_language': language,
}
```

### 3. Update Build Script

Edit `build_multilingual.sh` to include the new language:

```bash
echo "Step 4: Build French documentation..."
sphinx-build -D language=fr -b html . _build/html/fr
```

### 4. Translate Strings

Edit `.po` files in `locale/fr/LC_MESSAGES/` and add French translations.

## Translation Guidelines

### Best Practices

1. **Consistency**: Use consistent terminology throughout translations
2. **Technical Terms**: Keep technical terms like `VyraEntity`, `ROS2`, `Redis` untranslated
3. **Code Blocks**: Never translate code examples, only surrounding text
4. **Links**: Links are automatically handled by Sphinx, don't modify them

### Common Technical Terms (German)

| English | German | Notes |
|---------|--------|-------|
| Service | Service | Keep English |
| Topic | Topic | Keep English |
| Action | Action | Keep English |
| Request | Anfrage | Translate |
| Response | Antwort | Translate |
| Callback | Callback | Keep English |
| Module | Modul | Translate |
| Interface | Schnittstelle | Translate |
| Configuration | Konfiguration | Translate |

### Example Translation

```po
# Good ✅
msgid "The Core module forms the heart of the VYRA framework."
msgstr "Das Core-Modul bildet das Herz des VYRA-Frameworks."

# Bad ❌ (over-translated technical terms)
msgid "The Core module forms the heart of the VYRA framework."
msgstr "Das Kern-Modul bildet das Herz des VYRA-Rahmens."
```

## Troubleshooting

### Missing Translations

**Problem**: Some content not translated in built documentation.

**Solution**: 
1. Check `.po` file has non-empty `msgstr` for that string
2. Rebuild with `./build_multilingual.sh`
3. Clear browser cache

### Language Switcher Not Visible

**Problem**: Language selector doesn't appear in sidebar.

**Solution**:
1. Verify `_templates/sidebar/language.html` exists
2. Check `html_sidebars` in `conf.py` includes `'sidebar/language.html'`
3. Rebuild documentation

### Wrong Language Displayed

**Problem**: German build shows English text.

**Solution**:
1. Ensure building with `-D language=de` flag
2. Check `.mo` files are generated (Sphinx compiles `.po` → `.mo` automatically)
3. Verify `locale/de/LC_MESSAGES/*.po` files have translations

### Fuzzy Translations

**Problem**: `.po` file has `#, fuzzy` marker.

**Explanation**: Sphinx marked this translation as potentially outdated because source text changed slightly.

**Solution**:
1. Review the translation
2. Update if needed
3. Remove `#, fuzzy` line
4. Rebuild

## Resources

- [Sphinx Internationalization](https://www.sphinx-doc.org/en/master/usage/advanced/intl.html)
- [sphinx-intl Documentation](https://sphinx-intl.readthedocs.io/)
- [GNU gettext Manual](https://www.gnu.org/software/gettext/manual/)
- [Furo Theme Documentation](https://pradyunsg.me/furo/)

## Status

| File | English | German | Status |
|------|---------|--------|--------|
| index.rst | ✅ | ✅ | Complete |
| core.rst | ✅ | ✅ | Complete |
| helper.rst | ✅ | ✅ | Complete |
| interfaces_guide.rst | ✅ | 🔄 | Partial (main sections done) |
| storage.rst | ✅ | ⏳ | TODO |
| security/* | ✅ | ⏳ | TODO |
| com/* | ✅ | ⏳ | TODO |

Legend: ✅ Complete | 🔄 Partial | ⏳ TODO
