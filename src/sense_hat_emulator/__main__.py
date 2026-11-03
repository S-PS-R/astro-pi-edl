"""Launch the independent sensor GUI: python -m sense_hat_emulator."""

from . import SenseEmu


def main():
    SenseEmu().show_gui()


if __name__ == '__main__':
    main()
