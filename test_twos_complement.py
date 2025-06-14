#!/usr/bin/env python3
"""
Quick test to verify two's complement conversion
"""

def decode_if_negative(value: int, n_bytes: int = 4) -> int:
    """Decode value as two's complement if it appears to be a negative value interpreted as unsigned"""
    max_signed = (1 << (n_bytes * 8 - 1)) - 1  # Max positive value for signed integer
    if value > max_signed:
        # This is likely a negative value interpreted as unsigned
        return value - (1 << (n_bytes * 8))
    return value

# Test your specific value
test_value = 4294966982
decoded = decode_if_negative(test_value)

print(f"Raw value: {test_value}")
print(f"Decoded value: {decoded}")
print(f"\nVerification: {test_value} - 2^32 = {test_value} - {2**32} = {decoded}")

# Test some other common cases
print("\n--- Other Examples ---")
test_values = [
    4294966933,  # Your original problematic value
    4294966845,  # Another one you mentioned
    4294966928,  # Gripper value
    2147483647,  # Max positive (should stay positive)
    2147483648,  # First negative (-2147483648)
    4294967295,  # -1
]

for val in test_values:
    decoded = decode_if_negative(val)
    print(f"{val} → {decoded}")
