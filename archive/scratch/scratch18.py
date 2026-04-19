import scratch17
for X in np.linspace(0.1, 1.0, 100):
    p = scratch17.project(X, 0)
    if p and abs(p[1] - 368) < 5:
        print(f"y=368 corresponds to X={X:.3f}m")
for X in np.linspace(0.1, 5.0, 100):
    p = scratch17.project(X, 0)
    if p and abs(p[1] - 151) < 5:
        print(f"y=151 corresponds to X={X:.3f}m")

