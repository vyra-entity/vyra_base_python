# Example 07 - helper

This folder now targets full helper coverage by example mapping.

## Files

- `file_io_async_sync.py`
	- async/sync `FileReader` and `FileWriter` usage
- `helper_function_coverage.py`
	- `deep_merge`, `fuzzy_match`
	- async/sync file locks
	- required environment variable access (`get_env_required`)

## Coverage matrix

Covered helper modules and functions:

- `helper.func`
	- `deep_merge`
	- `fuzzy_match`
- `helper.file_lock`
	- `get_lock_for_file`
	- `release_lock_for_file`
	- `get_lock_for_file_sync`
	- `release_lock_for_file_sync`
- `helper.file_reader`
	- async/sync open helpers via `file_io_async_sync.py`
- `helper.file_writer`
	- async/sync write helpers via `file_io_async_sync.py`
- `helper.env_handler`
	- `get_env_required`

## Run

```bash
python examples/07_helper/file_io_async_sync.py
python examples/07_helper/helper_function_coverage.py
```
