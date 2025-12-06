from onnxruntime.quantization import quantize_dynamic, QuantType

# Ezt már letöltötted HuggingFace-ről:
fp32_model_path = "models/yolov4-tiny.onnx"

# Ezt fogjuk legenerálni:
int8_model_path = "models/yolov4-tiny-int8.onnx"

quantize_dynamic(
    model_input=fp32_model_path,
    model_output=int8_model_path,
    weight_type=QuantType.QInt8,   # súlyok int8
)

print("Kész, quantized modell mentve ide:", int8_model_path)
