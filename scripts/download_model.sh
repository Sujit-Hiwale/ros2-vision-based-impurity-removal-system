#!/bin/bash

MODEL_DIR="src/garlic_impurity_removal/config"
MODEL_FILE="garlic_detection.index"

mkdir -p $MODEL_DIR

if [ ! -f "$MODEL_DIR/$MODEL_FILE" ]; then
    echo "Downloading model from Hugging Face..."

    wget -O $MODEL_DIR/$MODEL_FILE \
    https://huggingface.co/Sujit-Hiwale/Garlic_anomaly_detection/resolve/main/garlic_detection.index

    echo "Download complete."
else
    echo "Model already exists. Skipping download."
fi
