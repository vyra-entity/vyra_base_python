"""
Security level matrix example
=============================

Demonstrates pure-python security helpers from vyra_base.security.security_levels
without requiring ROS runtime.
"""

from vyra_base.security.security_levels import SecurityLevel, AlgorithmId


def main() -> None:
    print("Security level overview:\n")

    for level in SecurityLevel:
        algorithm = AlgorithmId.for_security_level(level)
        print(
            f"- SL{level.value} {level.name:<18} | "
            f"algorithm={algorithm.value:<12} | "
            f"requires_hmac={level.requires_hmac()} | "
            f"requires_certificate={level.requires_certificate()}"
        )

    print("\nValidation helpers:")
    print("is_valid(4):", SecurityLevel.is_valid(4))
    print("is_valid(99):", SecurityLevel.is_valid(99))
    print("get_name(3):", SecurityLevel.get_name(3))


if __name__ == "__main__":
    main()
