"""K230 vision entry point.

Keep this code independent from the STM32 firmware project. Hardware control
should communicate through a documented protocol, not by importing firmware
files.
"""


def main() -> None:
    """Run the K230 vision task."""
    print("K230 vision module is ready.")


if __name__ == "__main__":
    main()
