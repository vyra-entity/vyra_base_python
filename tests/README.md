# vyra_base_python Test Suite

Comprehensive test suite for the vyra_base_python library, covering all modules with professional-grade unittest patterns.

## Test Coverage

This test suite provides extensive coverage for all vyra_base modules:

### Core Modules
- **test_redis_client.py** - Redis client with TLS, pub/sub, streaming functionality
- **test_storage.py** - Abstract storage interfaces and patterns
- **test_entity.py** - VyraEntity with ROS2 integration and state management
- **test_volatile.py** - Parameter management with Redis backend

### Helper Modules
- **test_helper.py** - Error handling, file operations, async utilities

### Communication Modules
- **test_communication.py** - DataSpace, VyraNode, callables, speakers, ROS2 integration

### System Modules
- **test_security.py** - Authentication, authorization, encryption patterns
- **test_database.py** - ORM, migrations, transactions, performance optimization
- **test_defaults.py** - Default entries, exceptions, configurations
- **test_interfaces.py** - Protocols, abstract base classes, contracts

## Running Tests

### Quick Test Run
```bash
# Run all tests with basic output
python run_tests.py

# Or use pytest directly
pytest tests/
```

### Coverage Analysis
```bash
# Run with coverage report
python run_tests.py --coverage

# Or with pytest
pytest tests/ --cov=src/vyra_base --cov-report=html --cov-report=term
```

### Parallel Execution
```bash
# Run tests in parallel for faster execution
pytest tests/ -n auto

# Or specify number of workers
pytest tests/ -n 4
```

### Specific Test Categories
```bash
# Run only unit tests
pytest tests/ -m "unit"

# Run only integration tests
pytest tests/ -m "integration"

# Run Redis-related tests
pytest tests/ -m "redis"

# Run ROS2-related tests
pytest tests/ -m "ros2"

# Skip slow tests
pytest tests/ -m "not slow"
```

## Test Structure

### Professional Patterns Used
- **Comprehensive Mocking**: External dependencies (Redis, ROS2) are properly mocked
- **Async Testing**: Full support for asyncio patterns with proper event loop management
- **State Management**: Singleton state reset between tests for isolation
- **Error Scenarios**: Extensive error condition testing with proper exception handling
- **Integration Patterns**: Real-world usage scenarios with multiple component interaction

### Test Fixtures
The `conftest.py` provides shared fixtures:
- `mock_redis_client` - Pre-configured Redis client mock
- `mock_ros2_node` - ROS2 node mock with standard methods
- `sample_state_entry` - StateEntry for state management tests
- `sample_module_entry` - ModuleEntry for module tests
- `temp_config_file` - Temporary configuration files

### Test Markers
- `@pytest.mark.unit` - Unit tests (default)
- `@pytest.mark.integration` - Integration tests
- `@pytest.mark.asyncio` - Async tests (auto-applied)
- `@pytest.mark.redis` - Redis-dependent tests
- `@pytest.mark.ros2` - ROS2-dependent tests
- `@pytest.mark.slow` - Long-running tests

## CI/CD Integration

### GitHub Actions Example
```yaml
name: Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - uses: actions/setup-python@v4
      with:
        python-version: '3.8'
    - name: Install dependencies
      run: |
        pip install -e .
        pip install pytest pytest-asyncio pytest-cov pytest-xdist pytest-timeout
    - name: Run tests
      run: python run_tests.py --coverage --parallel
    - name: Upload coverage
      uses: codecov/codecov-action@v3
```

### Jenkins Pipeline Example
```groovy
pipeline {
    agent any
    stages {
        stage('Test') {
            steps {
                sh 'python run_tests.py --coverage --xml-report'
            }
        }
    }
    post {
        always {
            publishTestResults testResultsPattern: 'test-results.xml'
            publishCoverageGoberturaReport 'coverage.xml'
        }
    }
}
```

## Test Configuration

### pytest.ini Configuration
```ini
[tool:pytest]
minversion = 6.0
addopts = 
    -ra 
    --strict-markers 
    --disable-warnings 
    --cov-fail-under=80
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
markers =
    unit: Unit tests
    integration: Integration tests
    redis: Redis-dependent tests
    ros2: ROS2-dependent tests
    slow: Long-running tests
timeout = 60
```

### Coverage Configuration
```ini
[tool:coverage.run]
source = src/vyra_base
omit = 
    */tests/*
    */test_*
    */__pycache__/*
    */venv/*

[tool:coverage.report]
exclude_lines =
    pragma: no cover
    def __repr__
    raise AssertionError
    raise NotImplementedError
```

## Test Development Guidelines

### Adding New Tests
1. Create test file following `test_<module_name>.py` pattern
2. Use appropriate test markers for categorization
3. Mock external dependencies properly
4. Include both positive and negative test cases
5. Test error conditions and edge cases
6. Add docstrings explaining test purpose

### Mock Guidelines
- Mock at the integration boundary (Redis, ROS2, file system)
- Use `unittest.mock.AsyncMock` for async methods
- Configure mocks to return realistic data
- Verify mock calls in tests when appropriate
- Reset mocks between tests using fixtures

### Async Test Guidelines
- Use `pytest.mark.asyncio` marker (auto-applied by conftest)
- Use `AsyncMock` for async dependencies
- Test both successful and error conditions
- Ensure proper cleanup in finally blocks
- Test concurrent scenarios where applicable

## Performance Considerations

### Test Execution Speed
- Parallel execution reduces total test time
- Mocks eliminate external dependencies
- Focused test markers allow selective execution
- Timeout configuration prevents hanging tests

### Resource Management
- Fixtures handle setup/teardown automatically
- Singleton state reset prevents test pollution
- Temporary files cleaned up automatically
- Memory usage optimized with proper mocking

## Troubleshooting

### Common Issues

1. **Import Errors**
   - Ensure src directory is in Python path
   - Check conftest.py configuration
   - Verify module structure matches imports

2. **Async Test Failures**
   - Check event loop configuration in conftest.py
   - Ensure AsyncMock usage for async methods
   - Verify proper await usage

3. **Mock Configuration**
   - Reset mock state between tests
   - Configure return values appropriately
   - Use side_effect for complex behaviors

4. **Coverage Issues**
   - Check coverage configuration
   - Ensure all code paths are tested
   - Add tests for error conditions

### Debug Commands
```bash
# Run with verbose output
pytest tests/ -v

# Run specific test with output
pytest tests/test_redis_client.py::test_redis_connection -s

# Debug failing test
pytest tests/test_redis_client.py::test_redis_connection --pdb

# Show test collection
pytest tests/ --collect-only
```

## Contributing

When contributing new tests:
1. Follow existing patterns and structure
2. Include comprehensive docstrings
3. Test both success and failure scenarios
4. Update this README if adding new test categories
5. Ensure tests pass in CI/CD environment

## Dependencies

Core testing dependencies:
- `pytest` - Test framework
- `pytest-asyncio` - Async test support
- `pytest-cov` - Coverage reporting
- `pytest-xdist` - Parallel execution
- `pytest-timeout` - Test timeouts
- `pytest-mock` - Enhanced mocking

Optional CI/CD dependencies:
- `pytest-html` - HTML reports
- `pytest-json-report` - JSON reports
- `pytest-xvfb` - Virtual display for GUI tests