# test_memory_usage.py
import psutil
import os
import time
from MA4_1_1 import approximate_pi  # Import the function to test

mem_usage = []
def memory_usage():
    """Return the current memory usage in MB."""
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 ** 2)  # Convert bytes to MB


def analyze_memory(func):
    """Decorator to analyze memory usage of a function."""

    def wrapper(*args, **kwargs):
        before_memory = memory_usage()
        start_time = time.time()  # Start timer
        result = func(*args, **kwargs)
        end_time = time.time()  # End timer
        after_memory = memory_usage()
        memory_used = after_memory - before_memory
        execution_time = end_time - start_time
        print(f"Memory used by '{func.__name__}': {memory_used:.2f} MB")
        mem_usage.append(memory_used)
        print(f"Execution time of '{func.__name__}': {execution_time:.4f} seconds")
        return result

    return wrapper


@analyze_memory
def test_approximate_pi(n):
    """Test approximate_pi with different numbers of dots."""
    approximate_pi(n)

def run_test_for_ma4_1():
    # dummy call
    approximate_pi(1000)
    dots = [1000, 10000, 100000]
    for n in dots:
        test_approximate_pi(n)
    return mem_usage
