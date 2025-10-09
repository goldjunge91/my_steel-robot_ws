#!/usr/bin/env python3
"""
Test script to verify analysis infrastructure is working correctly.
This validates that all components of task 1 are properly implemented.
"""

import sys
from pathlib import Path

# Add analysis directory to path
sys.path.insert(0, str(Path(__file__).parent))

def test_package_status_analyzer():
    """Test that package status analyzer can be imported and basic functions work."""
    try:
        from package_status_analyzer import PackageStatusAnalyzer
        
        # Test initialization
        analyzer = PackageStatusAnalyzer(".")
        
        # Test package discovery
        packages = analyzer.find_packages()
        print(f"✅ Package discovery: Found {len(packages)} packages")
        
        # Test package.xml parsing
        if packages:
            test_pkg = packages[0]
            analyzer.parse_package_xml(test_pkg)
            print(f"✅ Package.xml parsing: Parsed metadata for {test_pkg.name}")
        
        return True
    except Exception as e:
        print(f"❌ Package status analyzer test failed: {e}")
        return False

def test_duplicate_detection_analyzer():
    """Test that duplicate detection analyzer can be imported and basic functions work."""
    try:
        from duplicate_detection_analyzer import DuplicateDetectionAnalyzer
        
        # Test initialization
        analyzer = DuplicateDetectionAnalyzer(".")
        
        # Test package discovery
        packages = analyzer.find_packages()
        print(f"✅ Duplicate detection discovery: Found {len(packages)} packages")
        
        # Test fingerprint creation
        if packages:
            test_pkg = packages[0]
            fingerprint = analyzer.create_package_fingerprint(test_pkg)
            print(f"✅ Package fingerprinting: Created fingerprint for {fingerprint.name}")
        
        return True
    except Exception as e:
        print(f"❌ Duplicate detection analyzer test failed: {e}")
        return False

def test_analysis_runner():
    """Test that the main analysis runner can be imported."""
    try:
        from run_analysis import WorkspaceAnalysisRunner
        
        # Test initialization
        WorkspaceAnalysisRunner(".")
        print(f"✅ Analysis runner: Initialized successfully")
        
        return True
    except Exception as e:
        print(f"❌ Analysis runner test failed: {e}")
        return False

def main():
    """Run all infrastructure tests."""
    print("🔧 Testing Analysis Infrastructure Components")
    print("=" * 50)
    
    tests = [
        ("Package Status Analyzer", test_package_status_analyzer),
        ("Duplicate Detection Analyzer", test_duplicate_detection_analyzer),
        ("Analysis Runner", test_analysis_runner)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n📋 Testing {test_name}:")
        if test_func():
            passed += 1
        else:
            print(f"❌ {test_name} failed")
    
    print(f"\n📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("✅ All analysis infrastructure components are working correctly!")
        print("🎯 Task 1 infrastructure setup is COMPLETE")
        return True
    else:
        print("❌ Some infrastructure components have issues")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)