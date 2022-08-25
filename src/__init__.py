"""
An init file...yay
"""

try:
    from mavlite import (
        MavLink,
        UART
    )
except ImportError:
    from .mavlite import (
        MavLink,
        UART
    )
