"""
Example 09 — Async/Sync FileReader + FileWriter

Shows consistent usage of helper file I/O APIs:
- async JSON/YAML/TOML read/write
- sync JSON read/write
- lock-protected concurrent access
"""

import asyncio
from pathlib import Path

from vyra_base.helper.file_reader import FileReader
from vyra_base.helper.file_writer import FileWriter


async def main() -> None:
    base = Path("/tmp/vyra_base_example_file_io")
    base.mkdir(parents=True, exist_ok=True)

    json_file = base / "config.json"
    yaml_file = base / "config.yaml"
    toml_file = base / "config.toml"

    payload = {
        "module": "demo",
        "retry": 3,
        "enabled": True,
        "tags": ["example", "helper", "io"],
    }

    await FileWriter.write_json_file(json_file, payload)
    await FileWriter.write_yaml_file(yaml_file, payload)
    toml_file.write_text('[app]\nname = "demo"\nretries = 3\n', encoding="utf-8")

    json_data = await FileReader.open_json_file(json_file)
    yaml_data = await FileReader.open_yaml_file(yaml_file)
    toml_data = await FileReader.open_toml_file(toml_file)

    print("JSON:", json_data)
    print("YAML:", yaml_data)
    print("TOML:", toml_data)

    # sync path
    sync_file = base / "sync.json"
    FileWriter.write_json_file_sync(sync_file, {"mode": "sync", "ok": True})
    print("SYNC JSON:", FileReader.open_json_file_sync(sync_file))


if __name__ == "__main__":
    asyncio.run(main())
