import onnxruntime as ort

sess = ort.InferenceSession("src/mecanumbot_camera/models/ssd_mobilenet_v1_10.onnx")

print("Inputs:")
for inp in sess.get_inputs():
    print(inp.name, inp.shape, inp.type)

print("\nOutputs:")
for out in sess.get_outputs():
    print(out.name, out.shape, out.type)
