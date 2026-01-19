#!/bin/bash
# Build multilingual documentation locally
# This script builds both English and German versions of the documentation

set -e  # Exit on error

echo "==========================================="
echo "Building Multilingual VYRA Documentation"
echo "==========================================="

# Check if sphinx is installed
if ! command -v sphinx-build &> /dev/null; then
    echo "❌ Error: sphinx-build not found. Please install Sphinx:"
    echo "   pip install sphinx sphinx-intl sphinx-autodoc-typehints furo babel"
    exit 1
fi

# Navigate to docs directory
cd "$(dirname "$0")"

echo ""
echo "Step 1: Extract translatable strings (POT files)..."
sphinx-build -b gettext . _build/gettext

echo ""
echo "Step 2: Update PO files for German..."
sphinx-intl update -p _build/gettext -l de

echo ""
echo "Step 3: Build English documentation..."
sphinx-build -b html -D language=en . _build/html/en

echo ""
echo "Step 4: Build German documentation..."
sphinx-build -b html -D language=de . _build/html/de

echo ""
echo "Step 5: Create index redirect page..."
cat > _build/html/index.html << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Redirecting to VYRA Base Documentation</title>
    <meta http-equiv="refresh" content="0; url=en/index.html">
    <script>
      // Detect browser language and redirect accordingly
      var userLang = navigator.language || navigator.userLanguage;
      if (userLang.startsWith('de')) {
        window.location.href = 'de/index.html';
      } else {
        window.location.href = 'en/index.html';
      }
    </script>
    <style>
      body {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
        height: 100vh;
        margin: 0;
        background-color: #f5f5f5;
      }
      .container {
        text-align: center;
        padding: 2rem;
        background: white;
        border-radius: 8px;
        box-shadow: 0 2px 10px rgba(0,0,0,0.1);
      }
      h1 {
        color: #336790;
        margin-bottom: 1.5rem;
      }
      .language-buttons {
        display: flex;
        gap: 1rem;
        justify-content: center;
        margin-top: 1.5rem;
      }
      .btn {
        padding: 0.75rem 1.5rem;
        border: none;
        border-radius: 4px;
        font-size: 1rem;
        cursor: pointer;
        text-decoration: none;
        display: inline-block;
        transition: background-color 0.3s;
      }
      .btn-primary {
        background-color: #336790;
        color: white;
      }
      .btn-primary:hover {
        background-color: #2a5473;
      }
      .btn-secondary {
        background-color: #e0e0e0;
        color: #333;
      }
      .btn-secondary:hover {
        background-color: #d0d0d0;
      }
    </style>
</head>
<body>
    <div class="container">
      <h1>🌐 VYRA Base Documentation</h1>
      <p>Please select your preferred language:</p>
      <p>Bitte wählen Sie Ihre bevorzugte Sprache:</p>
      <div class="language-buttons">
        <a href="en/index.html" class="btn btn-primary">🇬🇧 English</a>
        <a href="de/index.html" class="btn btn-secondary">🇩🇪 Deutsch</a>
      </div>
    </div>
</body>
</html>
EOF

echo ""
echo "==========================================="
echo "✅ Build complete!"
echo "==========================================="
echo ""
echo "Documentation built in:"
echo "  - English: _build/html/en/index.html"
echo "  - German:  _build/html/de/index.html"
echo "  - Redirect: _build/html/index.html"
echo ""
echo "To view locally, run:"
echo "  python -m http.server -d _build/html 8000"
echo "Then open: http://localhost:8000"
echo ""
