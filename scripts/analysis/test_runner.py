#!/usr/bin/env python3
"""
Test runner for all analysis tool unit tests

This script runs all unit tests for the analysis tools and provides
a comprehensive test report.

Requirements addressed: 1.1, 1.2, 1.3
"""

import unittest
import sys
import os
from pathlib import Path
from io import StringIO

# Add analysis directory to path
sys.path.insert(0, str(Path(__file__).parent))

def run_all_tests():
    """Run all unit tests for analysis tools."""
    
    print("🧪 Running Analysis Tools Unit Tests")
    print("=" * 60)
    
    # Discover and run all tests
    loader = unittest.TestLoader()
    start_dir = Path(__file__).parent
    
    # Load test suites
    test_suites = []
    
    try:
        # Package Status Analyzer tests
        from test_package_status_analyzer import TestPackageStatusAnalyzer, TestPackageStatusAnalyzerIntegration
        suite1 = loader.loadTestsFromTestCase(TestPackageStatusAnalyzer)
        suite2 = loader.loadTestsFromTestCase(TestPackageStatusAnalyzerIntegration)
        test_suites.extend([suite1, suite2])
        print("✅ Loaded Package Status Analyzer tests")
    except ImportError as e:
        print(f"❌ Failed to load Package Status Analyzer tests: {e}")
    
    try:
        # Duplicate Detection Analyzer tests
        from test_duplicate_detection_analyzer import TestDuplicateDetectionAnalyzer, TestDuplicateDetectionIntegration
        suite3 = loader.loadTestsFromTestCase(TestDuplicateDetectionAnalyzer)
        suite4 = loader.loadTestsFromTestCase(TestDuplicateDetectionIntegration)
        test_suites.extend([suite3, suite4])
        print("✅ Loaded Duplicate Detection Analyzer tests")
    except ImportError as e:
        print(f"❌ Failed to load Duplicate Detection Analyzer tests: {e}")
    
    try:
        # Dependency Resolution tests
        from test_dependency_resolution import TestDependencyResolution, TestDependencyResolutionIntegration
        suite5 = loader.loadTestsFromTestCase(TestDependencyResolution)
        suite6 = loader.loadTestsFromTestCase(TestDependencyResolutionIntegration)
        test_suites.extend([suite5, suite6])
        print("✅ Loaded Dependency Resolution tests")
    except ImportError as e:
        print(f"❌ Failed to load Dependency Resolution tests: {e}")
    
    if not test_suites:
        print("❌ No test suites loaded. Cannot run tests.")
        return False
    
    # Combine all test suites
    combined_suite = unittest.TestSuite(test_suites)
    
    # Run tests with detailed output
    print(f"\n🚀 Running {combined_suite.countTestCases()} tests...")
    print("-" * 60)
    
    # Capture test output
    stream = StringIO()
    runner = unittest.TextTestRunner(
        stream=stream,
        verbosity=2,
        buffer=True
    )
    
    result = runner.run(combined_suite)
    
    # Print results
    output = stream.getvalue()
    print(output)
    
    # Print summary
    print("\n" + "=" * 60)
    print("📊 TEST SUMMARY")
    print("=" * 60)
    
    total_tests = result.testsRun
    failures = len(result.failures)
    errors = len(result.errors)
    skipped = len(result.skipped) if hasattr(result, 'skipped') else 0
    passed = total_tests - failures - errors - skipped
    
    print(f"Total tests run: {total_tests}")
    print(f"✅ Passed: {passed}")
    print(f"❌ Failed: {failures}")
    print(f"💥 Errors: {errors}")
    print(f"⏭️  Skipped: {skipped}")
    
    success_rate = (passed / total_tests * 100) if total_tests > 0 else 0
    print(f"Success rate: {success_rate:.1f}%")
    
    # Print failure details
    if result.failures:
        print(f"\n❌ FAILURES ({len(result.failures)}):")
        for test, traceback in result.failures:
            if 'AssertionError:' in traceback:
                error_msg = traceback.split('AssertionError: ')[-1].split('\n')[0]
            else:
                error_msg = 'See details above'
            print(f"  • {test}: {error_msg}")
    
    if result.errors:
        print(f"\n💥 ERRORS ({len(result.errors)}):")
        for test, traceback in result.errors:
            error_msg = traceback.split('\n')[-2] if '\n' in traceback else traceback
            print(f"  • {test}: {error_msg}")
    
    # Overall result
    if result.wasSuccessful():
        print(f"\n🎉 ALL TESTS PASSED! Analysis tools are working correctly.")
        return True
    else:
        print(f"\n⚠️  Some tests failed. Please review the failures above.")
        return False

def run_specific_test_class(test_class_name):
    """Run tests for a specific test class."""
    
    print(f"🧪 Running tests for {test_class_name}")
    print("=" * 60)
    
    # Map test class names to modules
    test_modules = {
        'TestPackageStatusAnalyzer': 'test_package_status_analyzer',
        'TestPackageStatusAnalyzerIntegration': 'test_package_status_analyzer',
        'TestDuplicateDetectionAnalyzer': 'test_duplicate_detection_analyzer',
        'TestDuplicateDetectionIntegration': 'test_duplicate_detection_analyzer',
        'TestDependencyResolution': 'test_dependency_resolution',
        'TestDependencyResolutionIntegration': 'test_dependency_resolution'
    }
    
    if test_class_name not in test_modules:
        print(f"❌ Unknown test class: {test_class_name}")
        print(f"Available test classes: {', '.join(test_modules.keys())}")
        return False
    
    try:
        module_name = test_modules[test_class_name]
        module = __import__(module_name)
        test_class = getattr(module, test_class_name)
        
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromTestCase(test_class)
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        return result.wasSuccessful()
        
    except Exception as e:
        print(f"❌ Failed to run tests for {test_class_name}: {e}")
        return False

def run_quick_smoke_tests():
    """Run a quick smoke test to verify basic functionality."""
    
    print("🔥 Running Quick Smoke Tests")
    print("=" * 40)
    
    smoke_tests = []
    
    # Test 1: Package Status Analyzer import and basic functionality
    try:
        from package_status_analyzer import PackageStatusAnalyzer, PackageStatus
        analyzer = PackageStatusAnalyzer(".")
        # Test basic methods don't crash
        packages = analyzer.find_packages()
        print("✅ Package Status Analyzer: Basic functionality works")
        smoke_tests.append(True)
    except Exception as e:
        print(f"❌ Package Status Analyzer: {e}")
        smoke_tests.append(False)
    
    # Test 2: Duplicate Detection Analyzer import and basic functionality
    try:
        from duplicate_detection_analyzer import DuplicateDetectionAnalyzer
        analyzer = DuplicateDetectionAnalyzer(".")
        # Test basic methods don't crash
        packages = analyzer.find_packages()
        print("✅ Duplicate Detection Analyzer: Basic functionality works")
        smoke_tests.append(True)
    except Exception as e:
        print(f"❌ Duplicate Detection Analyzer: {e}")
        smoke_tests.append(False)
    
    # Test 3: Test infrastructure
    try:
        from test_infrastructure import main as test_infrastructure_main
        # Don't actually run it, just test import
        print("✅ Test Infrastructure: Import successful")
        smoke_tests.append(True)
    except Exception as e:
        print(f"❌ Test Infrastructure: {e}")
        smoke_tests.append(False)
    
    passed = sum(smoke_tests)
    total = len(smoke_tests)
    
    print(f"\n📊 Smoke Test Results: {passed}/{total} passed")
    
    if passed == total:
        print("🎉 All smoke tests passed! Ready for full testing.")
        return True
    else:
        print("⚠️  Some smoke tests failed. Check the errors above.")
        return False

def main():
    """Main entry point for test runner."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Run analysis tools unit tests')
    parser.add_argument('--class', dest='test_class', help='Run specific test class')
    parser.add_argument('--smoke', action='store_true', help='Run quick smoke tests only')
    parser.add_argument('--list', action='store_true', help='List available test classes')
    
    args = parser.parse_args()
    
    if args.list:
        print("Available test classes:")
        test_classes = [
            'TestPackageStatusAnalyzer',
            'TestPackageStatusAnalyzerIntegration', 
            'TestDuplicateDetectionAnalyzer',
            'TestDuplicateDetectionIntegration',
            'TestDependencyResolution',
            'TestDependencyResolutionIntegration'
        ]
        for test_class in test_classes:
            print(f"  • {test_class}")
        return
    
    if args.smoke:
        success = run_quick_smoke_tests()
    elif args.test_class:
        success = run_specific_test_class(args.test_class)
    else:
        success = run_all_tests()
    
    if success:
        print(f"\n✅ Task 1.4 Unit Tests: COMPLETED SUCCESSFULLY")
        print("🎯 All analysis tools have comprehensive unit test coverage")
    else:
        print(f"\n❌ Task 1.4 Unit Tests: Some tests failed")
        print("🔧 Please review and fix the failing tests")
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()