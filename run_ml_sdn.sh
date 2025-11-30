#!/bin/bash
# Automated SDN ML Routing Workflow

echo "=========================================="
echo "SDN Smart Routing with ML - Automation"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Step 1: Compilation
echo -e "${BLUE}Step 1: Compiling OMNeT++ project...${NC}"
cd routing2 || exit

echo "Generating message files..."
opp_msgc -s _m.cc -s _m.h node/Packet.msg

echo "Cleaning previous build..."
make clean

echo "Compiling..."
make

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Compilation successful!${NC}"
else
    echo -e "${RED}✗ Compilation failed!${NC}"
    exit 1
fi
echo ""

# Step 2: Training Phase
echo -e "${BLUE}Step 2: Running training phase (collecting data)...${NC}"
echo "This will run for 200s simulation time..."
./routing2 -u Cmdenv -c NetSDN_ML_Training

if [ -f "sdn_dataset.csv" ]; then
    LINES=$(wc -l < sdn_dataset.csv)
    echo -e "${GREEN}✓ Dataset created: $LINES lines${NC}"
    
    if [ "$LINES" -lt 50 ]; then
        echo -e "${RED}⚠ Warning: Low sample count. Consider running longer.${NC}"
    fi
else
    echo -e "${RED}✗ Dataset not created!${NC}"
    exit 1
fi
echo ""

# Step 3: ML Training
echo -e "${BLUE}Step 3: Training ML models...${NC}"
python3 ../train_ml_model.py

if [ -f "sdn_best_model.pkl" ]; then
    echo -e "${GREEN}✓ ML model trained and saved!${NC}"
    echo "Generated files:"
    ls -lh sdn_best_model.pkl model_comparison.png confusion_matrix.png feature_importance.png
else
    echo -e "${RED}✗ ML training failed!${NC}"
    exit 1
fi
echo ""

# Step 4: Inference Phase
echo -e "${BLUE}Step 4: Running with ML routing...${NC}"
./routing2 -u Cmdenv -c NetSDN_ML_Inference

echo -e "${GREEN}✓ ML inference completed!${NC}"
echo ""

# Step 5: Results Analysis
echo -e "${BLUE}Step 5: Analyzing results...${NC}"
echo "Scalar results:"
scavetool export -f results/*.sca -o results_summary.csv

if [ -f "results_summary.csv" ]; then
    echo -e "${GREEN}✓ Results exported to results_summary.csv${NC}"
else
    echo -e "${RED}⚠ Scavetool not found or no results${NC}"
fi
echo ""

# Summary
echo "=========================================="
echo -e "${GREEN}WORKFLOW COMPLETED!${NC}"
echo "=========================================="
echo ""
echo "Generated files:"
echo "  📊 sdn_dataset.csv - Training dataset"
echo "  🧠 sdn_best_model.pkl - Trained ML model"
echo "  📈 model_comparison.png - Model accuracies"
echo "  📉 confusion_matrix.png - Prediction analysis"
echo "  🎯 feature_importance.png - Feature rankings"
echo "  📋 results_summary.csv - Simulation results"
echo ""
echo "Next steps:"
echo "  1. View plots: open *.png"
echo "  2. Analyze data: python3 analyze_results.py"
echo "  3. Run comparison: ./routing2 -u Qtenv -c NetSDN_ML_Comparison"
echo ""