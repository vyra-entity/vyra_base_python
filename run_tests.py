"""
Comprehensive test runner for vyra_base_python library
Executes all test suites with detailed reporting and coverage analysis
"""

import pytest
import sys
import os
from pathlib import Path


def main():
    """Run all tests with comprehensive configuration"""
    
    # Get the project root directory
    project_root = Path(__file__).parent
    
    # Configure pytest arguments
    pytest_args = [
        str(project_root / "tests"),  # Test directory
        "-v",                         # Verbose output
        "--tb=short",                 # Short traceback format
        "--strict-markers",           # Strict marker validation
        "--strict-config",            # Strict config validation
        "-p", "no:warnings",          # Disable warning summary (optional)
        "--maxfail=5",                # Stop after 5 failures
        "--durations=10",             # Show 10 slowest tests
    ]
    
    # Add coverage if available
    try:
        import pytest_cov
        pytest_args.extend([
            "--cov=vyra_base",
            "--cov-report=html:htmlcov",
            "--cov-report=term-missing",
            "--cov-fail-under=80"
        ])
        print("✅ Running tests with coverage analysis")
    except ImportError:
        print("⚠️  pytest-cov not available, running without coverage")
    
    # Add parallel execution if available
    try:
        import pytest_xdist
        pytest_args.extend(["-n", "auto"])
        print("✅ Running tests in parallel")
    except ImportError:
        print("⚠️  pytest-xdist not available, running sequentially")
    
    print(f"🚀 Starting comprehensive test suite for vyra_base_python")
    print(f"📁 Project root: {project_root}")
    print(f"🧪 Test directory: {project_root / 'tests'}")
    
    # Run the tests
    exit_code = pytest.main(pytest_args)
    
    # Print summary
    if exit_code == 0:
        print("✅ All tests passed successfully!")
    else:
        print(f"❌ Tests failed with exit code: {exit_code}")
    
    return exit_code


if __name__ == "__main__":
    sys.exit(main())