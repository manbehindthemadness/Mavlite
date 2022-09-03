"""
An init file...yay
"""

try:
    from mavlite import (
        MavLink,
        UART,
        waits
    )
except ImportError:
    from .mavlite import (
        MavLink,
        UART,
        waits
    )
