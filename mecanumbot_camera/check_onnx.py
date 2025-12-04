import onnxruntime as ort
import numpy as np

MODEL = "/ws/src/mecanumbot_camera/models/yolov8n.onnx"

print("Loading:", MODEL)
session = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])

print("\n=== INPUTS ===")
for inp in session.get_inputs():
    print("name:", inp.name, "\nshape:", inp.shape, "\ntype:", inp.type, "\n")

print("\n=== OUTPUTS ===")
for out in session.get_outputs():
    print("name:", out.name, "\nshape:", out.shape, "\ntype:", out.type, "\n")

# Try a dummy inference
print("\nRunning dummy inference...")
dummy = np.random.rand(1, 3, 640, 640).astype(np.float32)
res = session.run(None, {session.get_inputs()[0].name: dummy})

print("\n=== OUTPUT VALUES ===")
for i, r in enumerate(res):
    print(f"Output[{i}] → shape: {r.shape}")
