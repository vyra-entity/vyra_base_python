#!/usr/bin/env python3
"""
Quick validation test for multi-callback ActionServer implementation.

This test validates:
1. ActionServerDecorator class exists
2. Sub-decorators (on_goal, on_cancel, execute) work
3. Decorated methods include callback_type metadata
4. ActionBlueprint stores multiple callbacks
5. CallbackRegistry handles multi-callback binding
"""

import sys
import os

# Add src to path to use local development version
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src'))

import asyncio
from vyra_base.com.core.decorators import remote_actionServer, get_decorated_methods
from vyra_base.com.core import ActionBlueprint, CallbackRegistry


class TestComponent:
    """Test component with multi-callback ActionServer decorators."""
    
    @remote_actionServer.on_goal(name="test_action", auto_register=True)
    async def accept_goal(self, goal_request):
        """Goal acceptance callback."""
        return True
    
    @remote_actionServer.on_cancel(name="test_action")
    async def handle_cancel(self, goal_handle):
        """Cancellation callback."""
        return True
    
    @remote_actionServer.execute(name="test_action")
    async def execute_action(self, goal_handle):
        """Execute callback."""
        return {"status": "done"}


def test_decorator_class():
    """Test 1: Verify ActionServerDecorator class."""
    print("\n" + "="*60)
    print("TEST 1: ActionServerDecorator Class")
    print("="*60)
    
    # Check that remote_actionServer has the sub-decorator methods
    assert hasattr(remote_actionServer, 'on_goal'), "Missing on_goal sub-decorator"
    assert hasattr(remote_actionServer, 'on_cancel'), "Missing on_cancel sub-decorator"
    assert hasattr(remote_actionServer, 'execute'), "Missing execute sub-decorator"
    
    print("✅ ActionServerDecorator has all sub-decorators")
    return True


def test_decorated_methods():
    """Test 2: Verify decorated methods have correct metadata."""
    print("\n" + "="*60)
    print("TEST 2: Decorated Methods Metadata")
    print("="*60)
    
    component = TestComponent()
    decorated = get_decorated_methods(component)
    
    # Should have 3 action methods
    actions = decorated['actions']
    print(f"Found {len(actions)} decorated action methods")
    
    assert len(actions) == 3, f"Expected 3 actions, got {len(actions)}"
    
    # Check each method has callback_type
    callback_types = set()
    for action in actions:
        callback_type = action.get('callback_type')
        assert callback_type, f"Missing callback_type for {action['name']}"
        callback_types.add(callback_type)
        print(f"  - {action['name']} callback_type='{callback_type}'")
    
    # Should have all three types
    expected_types = {'on_goal', 'on_cancel', 'execute'}
    assert callback_types == expected_types, \
        f"Expected {expected_types}, got {callback_types}"
    
    print("✅ All decorated methods have correct callback_type metadata")
    return True


def test_blueprint_multi_callback():
    """Test 3: Verify ActionBlueprint stores multiple callbacks."""
    print("\n" + "="*60)
    print("TEST 3: ActionBlueprint Multi-Callback Storage")
    print("="*60)
    
    # Get blueprint from registry
    blueprint = CallbackRegistry.get_blueprint("test_action")
    assert blueprint, "Blueprint 'test_action' not found in registry"
    assert isinstance(blueprint, ActionBlueprint), \
        f"Expected ActionBlueprint, got {type(blueprint)}"
    
    print(f"Blueprint type: {type(blueprint).__name__}")
    
    # Check multi-callback storage
    assert hasattr(blueprint, '_callbacks'), "Blueprint missing _callbacks dict"
    assert isinstance(blueprint._callbacks, dict), \
        "_callbacks should be a dict"
    
    expected_keys = {'on_goal', 'on_cancel', 'execute'}
    actual_keys = set(blueprint._callbacks.keys())
    assert actual_keys == expected_keys, \
        f"Expected keys {expected_keys}, got {actual_keys}"
    
    print(f"Blueprint has _callbacks dict with keys: {list(blueprint._callbacks.keys())}")
    
    # Check callback binding methods
    assert hasattr(blueprint, 'bind_callback'), "Missing bind_callback method"
    assert hasattr(blueprint, 'get_callback'), "Missing get_callback method"
    assert hasattr(blueprint, 'is_bound'), "Missing is_bound method"
    assert hasattr(blueprint, 'is_fully_bound'), "Missing is_fully_bound method"
    
    print("✅ ActionBlueprint has multi-callback storage and methods")
    return True


def test_callback_binding():
    """Test 4: Verify callback binding works correctly."""
    print("\n" + "="*60)
    print("TEST 4: Multi-Callback Binding")
    print("="*60)
    
    component = TestComponent()
    
    # Bind callbacks using bind_decorated_callbacks
    from vyra_base.com.core.decorators import bind_decorated_callbacks
    results = bind_decorated_callbacks(component)
    
    print(f"Binding results: {len(results)} items")
    for name, success in results.items():
        status = "✅" if success else "❌"
        print(f"  {status} {name}")
    
    # Get blueprint and check binding status
    blueprint = CallbackRegistry.get_blueprint("test_action")
    
    # Check each callback type is bound
    for callback_type in ['on_goal', 'on_cancel', 'execute']:
        is_bound = blueprint.is_bound(callback_type)
        print(f"  {callback_type}: {'bound' if is_bound else 'NOT bound'}")
        assert is_bound, f"Callback '{callback_type}' should be bound"
    
    # Check fully bound
    assert blueprint.is_fully_bound(), "Blueprint should be fully bound"
    
    print("✅ All callbacks bound successfully")
    return True


def test_callback_retrieval():
    """Test 5: Verify callbacks can be retrieved."""
    print("\n" + "="*60)
    print("TEST 5: Callback Retrieval")
    print("="*60)
    
    blueprint = CallbackRegistry.get_blueprint("test_action")
    
    # Retrieve each callback
    on_goal_cb = blueprint.get_callback('on_goal')
    on_cancel_cb = blueprint.get_callback('on_cancel')
    execute_cb = blueprint.get_callback('execute')
    
    assert on_goal_cb is not None, "on_goal callback should not be None"
    assert on_cancel_cb is not None, "on_cancel callback should not be None"
    assert execute_cb is not None, "execute callback should not be None"
    
    print(f"  on_goal: {on_goal_cb.__name__ if hasattr(on_goal_cb, '__name__') else 'callable'}")
    print(f"  on_cancel: {on_cancel_cb.__name__ if hasattr(on_cancel_cb, '__name__') else 'callable'}")
    print(f"  execute: {execute_cb.__name__ if hasattr(execute_cb, '__name__') else 'callable'}")
    
    # Test backward compatibility property
    legacy_callback = blueprint.callback
    assert legacy_callback == execute_cb, \
        "Legacy .callback property should return execute callback"
    
    print("✅ All callbacks retrieved successfully")
    print("✅ Backward compatibility verified")
    return True


def run_all_tests():
    """Run all validation tests."""
    print("\n" + "#"*60)
    print("# Multi-Callback ActionServer Implementation Test")
    print("#"*60)
    
    tests = [
        test_decorator_class,
        test_decorated_methods,
        test_blueprint_multi_callback,
        test_callback_binding,
        test_callback_retrieval,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"❌ TEST FAILED: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    print(f"Passed: {passed}/{len(tests)}")
    print(f"Failed: {failed}/{len(tests)}")
    
    if failed == 0:
        print("\n🎉 ALL TESTS PASSED! Multi-callback implementation validated.")
        return 0
    else:
        print(f"\n❌ {failed} TEST(S) FAILED")
        return 1


if __name__ == "__main__":
    exit_code = run_all_tests()
    sys.exit(exit_code)
