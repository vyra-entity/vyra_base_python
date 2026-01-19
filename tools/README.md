# Build Tools

This directory contains build and deployment scripts for vyra_base.

## Scripts

### build_wheel.sh

Builds a Python wheel package using Poetry.

**Usage:**
```bash
./tools/build_wheel.sh
```

**What it does:**
- Runs `poetry build` to create wheel and source distribution
- Outputs wheel to `dist/` directory
- Shows success message with wheel filename

**Use this when:**
- You only want to build the wheel without installing
- Testing the build process
- Using `make build-wheel`

---

### build_auto.sh

Automated build and deployment script with version management.

**Usage:**
```bash
./tools/build_auto.sh
```

**What it does:**
1. **Increments build number** in `pyproject.toml` (e.g., `0.1.8+build.4` → `0.1.8+build.5`)
2. **Builds wheel** using `build_wheel.sh`
3. **Finds Python installation** automatically (tries `python3.12`, `python3.11`, `python3.10`, `python3`)
4. **Uninstalls old version** from system
5. **Installs new wheel** to system Python
6. **Copies wheel** to:
   - `../vyra_module_template/wheels/`
   - `../../VOS2_WORKSPACE/modules/v2_modulemanager_*/wheels/`
   - `../../VOS2_WORKSPACE/vyra_base_image/`

**Use this when:**
- Developing and testing locally
- Need to update all module dependencies
- Want automatic version management

**Output example:**
```
========================================
Vyra Base - Automated Build
========================================

Step 1: Incrementing build number...
  Version: 0.1.8+build.4 → 0.1.8+build.5
  ✅ Build number incremented

Step 2: Building wheel...
✅ Wheel built successfully: dist/vyra_base-0.1.8+build.5-py3-none-any.whl

Step 3: Finding Python installation...
  Found: python3.12 (Python 3.12.3)
  Using: python3.12

Step 4: Uninstalling old version...
  ✅ Old version uninstalled

Step 5: Installing new wheel...
  ✅ Wheel installed: vyra_base-0.1.8+build.5-py3-none-any.whl

Step 6: Copying wheel to module directories...
  ✅ Copied to vyra_module_template/wheels
  ✅ Copied to v2_modulemanager/wheels
  ✅ Copied to vyra_base_image

========================================
✅ Build Complete!
========================================
Version:    0.1.8+build.5
Wheel:      vyra_base-0.1.8+build.5-py3-none-any.whl
Python:     python3.12 (3.12.3)
========================================
```

---

## Makefile Targets

### build-wheel

Build wheel using `build_wheel.sh`.

```bash
make build-wheel
```

**Equivalent to:**
```bash
./tools/build_wheel.sh
```

### link-ros

Link ROS2 packages (for development).

```bash
make link-ros
```

---

## Version Management

### Version Format

Versions follow the format: `X.Y.Z+build.N`

- `X.Y.Z`: Semantic version (manually updated for releases)
- `build.N`: Automatic build number (incremented by `build_auto.sh`)

**Example progression:**
```
0.1.8+build.1
0.1.8+build.2
0.1.8+build.3
...
0.1.9+build.1  (after manual version bump)
```

### When to Update Version Numbers

- **Build number** (`+build.N`): Automatically incremented on every `build_auto.sh` run
- **Patch version** (`X.Y.Z`): Manually update for:
  - Bug fixes: `0.1.8` → `0.1.9`
  - New features: `0.1.9` → `0.2.0`
  - Breaking changes: `0.2.0` → `1.0.0`

### Resetting Build Number

After updating the semantic version, reset build number to 1:

```bash
# In pyproject.toml
version = "0.2.0+build.1"  # Start fresh after version bump
```

---

## Python Version Selection

`build_auto.sh` automatically finds the best Python 3 installation:

1. Tries `python3.12`
2. Falls back to `python3.11`
3. Falls back to `python3.10`
4. Falls back to `python3`
5. Exits with error if none found

**To use a specific Python version manually:**
```bash
# Build with specific Python
poetry build
/usr/local/bin/python3.12 -m pip install dist/vyra_base-*.whl
```

---

## Directory Structure

```
tools/
├── build_wheel.sh          # Basic wheel builder
├── build_auto.sh           # Automated build + deploy
├── ros/
│   └── link_ros_packages.sh
└── devel/
    ├── build_auto.sh       # Development version
    └── dict_py.py
```

---

## Troubleshooting

### "Version format not recognized"

Ensure `pyproject.toml` has correct version format:
```toml
version = "0.1.8+build.4"
```

### "No wheel file found"

Poetry build failed. Check:
```bash
poetry install
poetry build
```

### "No Python 3 installation found"

Install Python 3:
```bash
sudo apt install python3
```

### Wheel not copied to modules

Check that directories exist:
- `../vyra_module_template/wheels/`
- `../../VOS2_WORKSPACE/modules/v2_modulemanager_*/wheels/`

### Permission errors

Use `--break-system-packages` flag (already in script) or virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
./tools/build_auto.sh
```

---

## Best Practices

1. **Development workflow:**
   ```bash
   # Make changes to code
   ./tools/build_auto.sh  # Build, install, deploy
   # Test in modules
   ```

2. **Release workflow:**
   ```bash
   # Update version in pyproject.toml
   version = "0.2.0+build.1"
   
   # Build release
   poetry build
   
   # Publish to PyPI (if applicable)
   poetry publish
   ```

3. **Testing without deployment:**
   ```bash
   make build-wheel  # Only builds, doesn't install
   ```

---

## See Also

- Poetry documentation: https://python-poetry.org/docs/
- Python packaging: https://packaging.python.org/
- Semantic versioning: https://semver.org/
