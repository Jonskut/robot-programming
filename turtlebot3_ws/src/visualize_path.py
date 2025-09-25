import csv
import matplotlib.pyplot as plt
from math import sqrt

def load_path(filename):
    path = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            theta = float(row.get('theta', 0.0))
            path.append((x, y, theta))
    return path

def compute_errors(ref_path, real_path):
    errors = []
    for ax, ay, _ in real_path:
        # Find closest reference point
        min_err = float('inf')
        for rx, ry, _ in ref_path:
            err = sqrt((rx - ax)**2 + (ry - ay)**2)
            if err < min_err:
                min_err = err
        errors.append(min_err)
    return errors

def main():
    import sys
    if len(sys.argv) < 3:
        print("Usage: python3 visualize_path.py reference.csv actual.csv")
        return

    ref_path = load_path(sys.argv[1])
    real_path = load_path(sys.argv[2])
    errors = compute_errors(ref_path, real_path)

    # Plot paths
    plt.figure(figsize=(8, 8))
    plt.plot([p[0] for p in ref_path], [p[1] for p in ref_path], 'b-', label='Reference Path')
    plt.plot([p[0] for p in real_path], [p[1] for p in real_path], 'r--', label='Actual Path')  # Dotted line
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Path Comparison')
    plt.axis('equal')
    plt.grid()

    # Plot error
    plt.figure()
    plt.plot(errors, 'k-')
    plt.xlabel('Step')
    plt.ylabel('Tracking Error (m)')
    plt.title('Tracking Error Over Time')
    plt.grid()

    print(f"Mean error: {sum(errors)/len(errors):.3f} m")
    print(f"Max error: {max(errors):.3f} m")

    plt.show()

if __name__ == '__main__':
    main()