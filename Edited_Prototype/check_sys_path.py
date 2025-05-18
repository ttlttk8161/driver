import sys
import os
print("sys.path:")
for p in sys.path:
    print(p)
print("\nPYTHONPATH environment variable:")
print(os.environ.get('PYTHONPATH'))