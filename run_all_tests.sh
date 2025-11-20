#!/bin/bash

echo "========================================="
echo "     RUNNING ALL NAVIGATION TESTS"
echo "========================================="

# Path to test directory
TEST_DIR="src/navigation_assignment/test"

# List of test scripts (add more here if needed)
TEST_FILES=(
    "test_path_smoothing.py"
    "test_trajectory_generator.py"
)

# Create results directory
RESULTS_DIR="test_results"
mkdir -p $RESULTS_DIR

echo ""
echo "[INFO] Saving plots & outputs into: $RESULTS_DIR/"
echo ""

# Run each test file
for test_file in "${TEST_FILES[@]}"; do
    echo "-----------------------------------------"
    echo " Running: $test_file"
    echo "-----------------------------------------"
    
    python3 "$TEST_DIR/$test_file"

    # Move any generated files into results folder
    mv *.png $RESULTS_DIR/ 2>/dev/null
done

echo ""
echo "========================================="
echo " ALL TESTS COMPLETED"
echo "========================================="
echo ""
echo "Results saved in: $RESULTS_DIR/"
